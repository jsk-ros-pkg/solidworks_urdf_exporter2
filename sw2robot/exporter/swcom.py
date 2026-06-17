"""SolidWorks COM connection helpers.

KEY FACTS about this machine (discovered empirically):

* The user's running SolidWorks does NOT register itself in the COM Running
  Object Table, so ``win32com.client.GetActiveObject("SldWorks.Application")``
  fails with ``MK_E_UNAVAILABLE`` and we cannot attach to the live session.
* Every ``Dispatch("SldWorks.Application")`` spawns a *new* SolidWorks process
  (the class object is single-use).  Therefore this module always creates and
  OWNS its own instance and is responsible for shutting it down (``ExitApp``)
  so it never leaks empty instances.
* We open the target file **read-only** and **never save** it, so the user's
  open document is never at risk.
* The COM client must be an **x64** Python to match SolidWorks (the project is
  pinned to a uv-managed x64 interpreter via ``.python-version``).
"""

from __future__ import annotations

import os
import shutil
import sys
import tempfile

# pywin32 is Windows-only and only needed for the SolidWorks `extract` phase.
# Guard the imports so the SW-free `build` path (model/urdf_writer/sw2robot.editor.ui)
# imports on any OS / a PC without pywin32 -- the COM calls below only run when
# `SolidWorks()`/`as_iface()` are actually used (i.e. during extract).
try:
    import pythoncom
    import win32com.client
    from win32com.client import gencache
    _HAVE_WIN32 = True
except ImportError:  # pragma: no cover - non-Windows / no pywin32
    pythoncom = win32com = gencache = None
    _HAVE_WIN32 = False

# SolidWorks 2024 type libraries (LIBID, lcid, major, minor) -- generated once
# via ``python -m win32com.client.makepy <sldworks.tlb>`` etc.  Loading them
# explicitly is what makes Dispatch return *typed* objects so that methods with
# [out] parameters (GetRemainingDOFs) and overloads (SaveAs4) marshal correctly.
# Plain dynamic dispatch cannot call those.  major=32 == SolidWorks 2024.
_TYPELIBS = [
    ("{83A33D31-27C5-11CE-BFD4-00400513BB57}", 0, 32, 0),  # sldworks
    ("{4687F359-55D0-4CD3-B6CF-2EB42C11F989}", 0, 32, 0),  # swconst
    ("{C71C31CD-898C-11D4-AEF6-00C04F603FAF}", 0, 32, 0),  # swpublished
]
_sld_module = None


def _prepare_gencache():
    """Make win32com able to GENERATE makepy wrappers at runtime.

    In a PyInstaller-frozen exe the default gen_py output dir is the read-only
    bundle (_MEIPASS) and ``gencache.is_readonly`` defaults True, so
    ``EnsureDispatch`` silently falls back to a LATE-bound object that cannot
    reach typelib-only methods such as ``OpenDoc6`` (the "AttributeError:
    SldWorks.Application.OpenDoc6" seen only in the frozen build).  Redirect
    gen_py to a writable temp dir and allow generation.  Idempotent; safe on a
    normal (non-frozen) run where it is essentially a no-op."""
    if gencache is None:
        return
    try:
        import win32com
        if getattr(sys, "frozen", False):
            gen = os.path.join(tempfile.gettempdir(), "sw2robot_gen_py")
            os.makedirs(gen, exist_ok=True)
            win32com.__gen_path__ = gen
            win32com.__build_path__ = gen
            try:        # repoint the package too, whatever the import order was
                import win32com.gen_py as _gp
                _gp.__path__ = [gen]
            except Exception:
                pass
        gencache.is_readonly = False
        gencache.GetGeneratePath()      # creates the package + __init__ if absent
    except Exception as e:
        print(f"  [swcom] gencache prep warning: {e!r}")


def ensure_typelibs():
    """Load the makepy-generated SolidWorks typelibs; return the sldworks module.

    The sldworks module is what we use to wrap any object that still comes back
    as a dynamic ``CDispatch`` (many SW methods are typed ``LPDISPATCH`` so
    win32com cannot auto-wrap their return value)."""
    global _sld_module
    if _sld_module is not None:
        return _sld_module
    _prepare_gencache()
    mod = _load_modules()
    if mod is None:
        # First run on this machine: generate the wrappers from the .tlb files.
        _makepy_solidworks()
        mod = _load_modules()
    _sld_module = mod
    return mod


def _load_modules():
    mod = None
    for libid, lcid, major, minor in _TYPELIBS:
        try:
            m = gencache.EnsureModule(libid, lcid, major, minor)
            if libid.startswith("{83A33D31"):
                mod = m
        except Exception:
            pass
    return mod


def _makepy_solidworks():
    """Locate sldworks.tlb (+swconst/swpublished) and run makepy on them."""
    import glob

    from win32com.client import makepy
    roots = [r"C:\Program Files\SOLIDWORKS Corp",
             r"C:\Program Files (x86)\SOLIDWORKS Corp"]
    for tlbname in ("sldworks.tlb", "swconst.tlb", "swpublished.tlb"):
        hit = None
        for root in roots:
            matches = glob.glob(os.path.join(root, "**", tlbname),
                                recursive=True)
            if matches:
                hit = matches[0]
                break
        if hit:
            try:
                makepy.GenerateFromTypeLibSpec(hit)
                print(f"  generated typelib wrapper for {tlbname}")
            except Exception as e:
                print(f"  makepy failed for {tlbname}: {e!r}")


def as_iface(obj, iface_name):
    """Wrap a (possibly dynamic) COM object as a typed SolidWorks interface.

    Returns the typed wrapper so methods with [out] params work, or ``obj``
    unchanged if wrapping is not possible.  ``IComponent2``, ``IModelDoc2``,
    ``IAssemblyDoc``, ``IModelDocExtension``, ``IMate2``, ``IMateEntity2``,
    ``IFeature``, ``IMathTransform`` are the ones we need."""
    if obj is None:
        return None
    mod = ensure_typelibs()
    if mod is None:
        return obj
    cls = getattr(mod, iface_name, None)
    if cls is None:
        return obj
    # The generated DispatchBaseClass wants a raw pythoncom PyIDispatch, NOT a
    # win32com CDispatch wrapper.  SolidWorks dual interfaces expose every
    # method's dispid through the one IDispatch, so no QueryInterface is needed.
    raw = getattr(obj, "_oleobj_", obj)
    try:
        return cls(raw)
    except Exception:
        return obj

# swDocumentTypes_e
SW_DOC_PART = 1
SW_DOC_ASSEMBLY = 2
SW_DOC_DRAWING = 3

# swOpenDocOptions_e
SW_OPEN_SILENT = 1
SW_OPEN_READONLY = 4

# swSaveAsOptions_e
SW_SAVEAS_SILENT = 1
SW_SAVEAS_COPY = 2  # save a copy without making it the active/dirty document


def is_com(val):
    return hasattr(val, "_oleobj_")


def safe_prop(obj, name):
    """Read ``obj.name`` whether it is a property or a no-arg method; None on fail."""
    try:
        val = getattr(obj, name)
    except Exception:
        return None
    if is_com(val):
        return val
    if callable(val):
        try:
            return val()
        except Exception:
            return None
    return val


def safe_call(obj, name, *args):
    try:
        attr = getattr(obj, name)
        if callable(attr) and not is_com(attr):
            return attr(*args)
        return attr
    except Exception:
        return None


def byref_long(initial=0):
    """A VT_I4|VT_BYREF VARIANT for SolidWorks [out] long parameters."""
    return win32com.client.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, initial)


def doc_type_for(path):
    ext = os.path.splitext(path)[1].lower()
    if ext == ".sldasm":
        return SW_DOC_ASSEMBLY
    if ext == ".slddrw":
        return SW_DOC_DRAWING
    return SW_DOC_PART


class SolidWorksUnavailable(RuntimeError):
    """SolidWorks could not be reached/started -- not installed, no running
    instance to attach to, or (most often) it launched but cannot check out a
    license because the license server is unreachable.  Raised instead of
    caching a dead instance that would never recover even once the license is
    restored."""


class SolidWorks:
    """Owns a private SolidWorks instance; opens docs read-only; never saves.

    Use as a context manager so the instance is always shut down::

        with SolidWorks(visible=False) as sw:
            doc = sw.open_readonly(path)
            ...
    """

    def __init__(self, visible=False, attach=False):
        """``attach=True``: connect to the USER'S RUNNING SolidWorks via
        GetActiveObject instead of spawning a private instance.  In that mode
        shutdown() leaves the application and its documents untouched."""
        self._attached = attach
        if attach:
            try:
                pythoncom.CoInitialize()
            except Exception:
                pass
            ensure_typelibs()
            try:
                self.app = win32com.client.GetActiveObject(
                    "SldWorks.Application")
            except Exception as e:
                raise SolidWorksUnavailable(
                    "no running SolidWorks to attach to -- start SolidWorks "
                    "(and check its license) first.") from e
            self.visible = True
            self._open_docs = []
            self._tempdirs = []
            if not self._responds():     # attached app is the USER's: don't quit
                raise SolidWorksUnavailable(
                    "the running SolidWorks is not responding (license "
                    "issue?).")
            return
        # Each Dispatch creates a brand-new process that we own outright.
        # Use the makepy/gencache typelib wrappers so that methods with [out]
        # parameters (GetRemainingDOFs, Extension.SaveAs, ...) and overloads
        # are marshalled correctly -- plain dynamic Dispatch raises
        # DISP_E_PARAMNOTOPTIONAL / DISP_E_TYPEMISMATCH on those.
        # pythoncom normally CoInitializes the importing thread, but some
        # launch contexts (sandboxed shells) reach here uninitialized --
        # calling it again is harmless (returns S_FALSE).
        try:
            pythoncom.CoInitialize()
        except Exception:
            pass
        ensure_typelibs()  # load typelibs first so Dispatch returns typed
        try:
            self.app = win32com.client.gencache.EnsureDispatch(
                "SldWorks.Application")
        except Exception:
            # benign here: with the typelib loaded (ensure_typelibs) the dynamic
            # object still resolves OpenDoc6 et al.  Only the frozen build, where
            # makepy can't generate, ends up genuinely late-bound -- handled next.
            self.app = win32com.client.Dispatch("SldWorks.Application")
        # A LATE-bound object that lacks OpenDoc6 cannot reach typelib-only
        # methods (the frozen-exe failure).  Force-generate the wrapper now that
        # gencache is writable and upgrade the live object to early binding.
        if not hasattr(self.app, "OpenDoc6"):
            print("  [swcom] dispatch is late-bound (no OpenDoc6); generating "
                  "the typelib wrapper ...")
            try:
                _prepare_gencache()
                self.app = win32com.client.gencache.EnsureDispatch(self.app)
            except Exception as e:
                print(f"  [swcom] could not upgrade to early binding: {e!r}")
            if not hasattr(self.app, "OpenDoc6"):
                print("  [swcom] WARNING: OpenDoc6 still unavailable -- makepy "
                      "could not generate the typelib wrapper (frozen build "
                      "with no writable gen_py?)")
        try:
            self.app.Visible = visible
        except Exception:
            pass
        self.visible = visible
        self._open_docs = []
        self._tempdirs = []
        # A SolidWorks that launched but can't acquire a license comes back as
        # a COM object that fails every real call.  Catch a wholly unresponsive
        # instance here -- with a clear license warning -- and tear it down, so
        # a dead/zombie process is never cached and handed to the next job.
        if not self._responds():
            try:
                self.shutdown()
            except Exception:
                pass
            raise SolidWorksUnavailable(
                "SolidWorks launched but is not responding -- it most likely "
                "cannot check out a license (is the license server "
                "reachable?). Fix the license and retry; nothing was cached.")

    def _responds(self):
        """True if the app answers a trivial, license-free call.  Lenient on
        purpose: a healthy instance always answers ``Visible`` (or, failing
        that, ``RevisionNumber``); only a wholly dead/zombie object fails both,
        so this never false-rejects a working SolidWorks."""
        if self.app is None:
            return False
        if safe_prop(self.app, "Visible") is not None:
            return True
        return safe_call(self.app, "RevisionNumber") not in (None, "")

    # -- document handling ------------------------------------------------
    def open_copy(self, path):
        """Copy ``path`` to a temp dir and open the COPY full (read-write).

        We never open or modify the user's original file.  Crucially we do NOT
        use the read-only open flag: opening an assembly read-only makes
        SolidWorks load it in a degraded graphics-only mode (feature tree of
        size 1, no mates, GetModelDoc2 returns the assembly).  Opening the
        throw-away copy read-write loads it fully (mates, per-part docs, DOFs).
        Nothing is ever saved -- ``shutdown`` discards all changes and deletes
        the temp copy.

        External part references (absolute paths, e.g. on a shared drive) still
        resolve from the copy.  Returns the opened ModelDoc2.
        """
        path = os.path.abspath(path)
        if not os.path.exists(path):
            raise FileNotFoundError(path)
        # A temp copy loses SolidWorks' "same folder as the assembly"
        # reference-resolution rule; assemblies whose STORED part paths are
        # stale then open with almost everything unresolved (home_drone
        # came up as a single component).  Register the original's folder
        # family as document search paths so rule (2) replaces rule (4).
        import time as _time
        t0 = _time.time()
        self._register_search_folders(path)
        tmpdir = tempfile.mkdtemp(prefix="sw2urdf_")
        self._tempdirs.append(tmpdir)
        tmp = os.path.join(tmpdir, os.path.basename(path))
        shutil.copy2(path, tmp)
        t1 = _time.time()

        errors = byref_long()
        warnings = byref_long()
        doc = self.app.OpenDoc6(tmp, doc_type_for(tmp), SW_OPEN_SILENT, "",
                                errors, warnings)
        t2 = _time.time()
        if doc is None:
            raise RuntimeError(
                f"OpenDoc6 failed for {tmp} "
                f"(errors={errors.value}, warnings={warnings.value})")
        self._open_docs.append(doc)
        t3 = t2
        if doc_type_for(tmp) == SW_DOC_ASSEMBLY:
            asm = as_iface(doc, "IAssemblyDoc")
            try:
                asm.ResolveAllLightWeightComponents(True)
            except Exception:
                pass
            t3 = _time.time()
            # ForceRebuild3 was added for stale as-saved poses (propellers
            # floating off motors), but it was MEASURED to be a no-op for
            # that bug (the real fix is _snap_unsolved_mates at build time)
            # while costing minutes on shared-drive assemblies (662 s on a
            # 6-part module).  Off by default; SW2URDF_REBUILD=1 restores it.
            if os.environ.get("SW2URDF_REBUILD") == "1":
                try:
                    md = as_iface(doc, "IModelDoc2")
                    md.ForceRebuild3(False)
                except Exception as e:
                    print(f"      WARN: ForceRebuild3 failed ({e!r}); "
                          f"component positions may be stale")
        t4 = _time.time()
        print(f"      open timing: copy {t1 - t0:.1f}s | OpenDoc6 "
              f"{t2 - t1:.1f}s | resolve {t3 - t2:.1f}s | rebuild "
              f"{t4 - t3:.1f}s")
        return doc

    def _register_search_folders(self, src_path):
        """Add the source assembly's folder, its parent and the parent's
        sub-folders (parts/, commercial/, ...) to the reference search
        paths of this SolidWorks session.

        The previous user settings are captured ONCE and restored in
        shutdown() -- SolidWorks persists these preferences to the registry
        on exit, so leaving ours behind would poison the user's own
        interactive sessions (same-named parts from another project could
        silently resolve through our folders)."""
        try:
            from win32com.client import constants
            if not hasattr(self, "_saved_search_prefs"):
                try:
                    self._saved_search_prefs = (
                        self.app.GetSearchFolders(constants.swDocumentType),
                        bool(self.app.GetUserPreferenceToggle(
                            constants.swUseFolderSearchRule)))
                except Exception:
                    self._saved_search_prefs = None
            src_dir = os.path.dirname(src_path)
            parent = os.path.dirname(src_dir)
            cand = [src_dir, parent]
            try:
                for d in sorted(os.listdir(parent)):
                    p = os.path.join(parent, d)
                    if os.path.isdir(p) and p != src_dir:
                        cand.append(p)
            except Exception:
                pass
            folders = ";".join(dict.fromkeys(cand))
            try:
                self.app.SetUserPreferenceToggle(
                    constants.swUseFolderSearchRule, True)
            except Exception:
                pass
            self.app.SetSearchFolders(constants.swDocumentType, folders)
            print(f"      reference search folders set ({len(cand)} dirs "
                  f"around {os.path.basename(src_dir)})")
        except Exception as e:
            print(f"      WARN: could not set search folders ({e!r}); "
                  f"stale part references may not resolve")

    def _restore_search_folders(self):
        """Put the user's reference-search preferences back (see
        _register_search_folders)."""
        saved = getattr(self, "_saved_search_prefs", None)
        if saved is None or self.app is None:
            return
        try:
            from win32com.client import constants
            folders, toggle = saved
            self.app.SetSearchFolders(constants.swDocumentType, folders or "")
            self.app.SetUserPreferenceToggle(
                constants.swUseFolderSearchRule, toggle)
            del self._saved_search_prefs
        except Exception:
            pass

    def close_doc(self, doc):
        title = safe_prop(doc, "GetTitle")
        if title:
            try:
                self.app.CloseDoc(title)
            except Exception:
                pass

    # -- lifecycle --------------------------------------------------------
    def shutdown(self):
        """Close all docs WITHOUT saving and terminate OUR instance only.

        In attach mode the application belongs to the USER: never close
        documents or exit -- just drop our reference."""
        self._restore_search_folders()
        if getattr(self, "_attached", False):
            self.app = None
            return
        try:
            # True == include unsaved; we opened read-only so nothing is dirty,
            # but this guarantees no save prompt blocks the headless run.
            self.app.CloseAllDocuments(True)
        except Exception:
            pass
        try:
            self.app.ExitApp()
        except Exception:
            pass
        self.app = None
        for d in self._tempdirs:
            shutil.rmtree(d, ignore_errors=True)
        self._tempdirs = []

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.shutdown()
        return False
