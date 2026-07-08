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

# SolidWorks type libraries (stable LIBIDs; generated via makepy).  Loading
# them is what makes Dispatch return *typed* objects so methods with [out]
# parameters (GetRemainingDOFs) and overloads (SaveAs4) marshal correctly --
# plain dynamic dispatch cannot call those.
#
# The typelib VERSION (major.minor) differs per SolidWorks release -- major ==
# release year - 1992 (2024 -> 32, 2025 -> 33, ...) -- so it is DETECTED at
# runtime from the registry (with a year-range fallback) instead of pinned to
# one release.  Development/CI is on SolidWorks 2024; if the installed version's
# typelib cannot be loaded, we raise a clear error pointing at the maintainer.
_TYPELIBS = {
    "sldworks":    "{83A33D31-27C5-11CE-BFD4-00400513BB57}",
    "swconst":     "{4687F359-55D0-4CD3-B6CF-2EB42C11F989}",
    "swpublished": "{C71C31CD-898C-11D4-AEF6-00C04F603FAF}",
}
_TESTED_MAJOR = 32                  # SolidWorks 2024 (year - 1992); the verified one
_ISSUES_URL = "https://github.com/iory/sw2robot/issues"
_sld_module = None


def _registered_typelib_versions(libid):
    """(major, minor) versions registered for ``libid``, newest first.

    The TypeLib registry stores version components in HEX (the COM convention),
    which is exactly what ``EnsureModule`` wants as integers, so e.g. the key
    ``20.0`` parses to ``(32, 0)`` = SolidWorks 2024."""
    out = []
    try:
        import winreg
        with winreg.OpenKey(winreg.HKEY_CLASSES_ROOT,
                            rf"TypeLib\{libid}") as k:
            i = 0
            while True:
                try:
                    sub = winreg.EnumKey(k, i)
                except OSError:
                    break
                i += 1
                try:
                    maj, _, minr = sub.partition(".")
                    out.append((int(maj, 16), int(minr or "0", 16)))
                except ValueError:
                    pass
    except Exception:
        pass
    out.sort(reverse=True)
    return out


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


def _version_candidates(libid):
    """Versions to try for ``libid``, in order:

    1. the VERIFIED major (SolidWorks 2024) if it is registered -- so the
       tested dev/CI machine keeps using the proven typelib;
    2. the remaining registered versions, newest first (other environments);
    3. a year-range fallback (major == year - 1992) if the registry read came
       up empty or in an unexpected encoding."""
    registered = _registered_typelib_versions(libid)
    ordered = [v for v in registered if v[0] == _TESTED_MAJOR]   # (1)
    for v in registered:                                         # (2)
        if v not in ordered:
            ordered.append(v)
    for y in range(2030, 2019, -1):                              # (3) 2030..2020
        v = (y - 1992, 0)
        if v not in ordered:
            ordered.append(v)
    return ordered


def _load_modules():
    """Load the makepy wrappers for whatever SolidWorks version is installed,
    auto-detecting each typelib's version.  Returns the sldworks module, or
    None if it could not be loaded for any candidate version."""
    if gencache is None:
        return None
    mod = None
    for name, libid in _TYPELIBS.items():
        for major, minor in _version_candidates(libid):
            try:
                m = gencache.EnsureModule(libid, 0, major, minor)
            except Exception:
                continue                      # not this version -- try the next
            if m is not None:
                if name == "sldworks":
                    mod = m
                break                         # this typelib loaded; next typelib
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


def open_doc6(app, path, dtype, options=SW_OPEN_SILENT):
    """``IModelDoc2.OpenDoc6`` -> ``(doc, error_code, warning_code)``, tolerant of
    the way win32com marshals its two ``[out]`` long params across versions.

    The historical approach passes ``VARIANT(VT_BYREF|VT_I4)`` objects; pywin32
    >= ~311 then raises ``TypeError: int() argument ... not 'VARIANT'`` while
    coercing them inside ``InvokeTypes``.  Fall back to passing plain ints -- the
    gen_py wrapper builds the byref itself and returns the updated values
    alongside the document.  (A real COM failure -- e.g. an RPC disconnect when
    SolidWorks dies -- is NOT a TypeError, so it propagates to the caller.)"""
    try:
        errors, warnings = byref_long(), byref_long()
        doc = app.OpenDoc6(path, dtype, options, "", errors, warnings)
        return doc, errors.value, warnings.value
    except TypeError:
        res = app.OpenDoc6(path, dtype, options, "", 0, 0)
        if isinstance(res, (tuple, list)):
            vals = [*res, 0, 0, 0]
            return vals[0], vals[1], vals[2]
        return res, 0, 0


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


class SolidWorksTypelibError(SolidWorksUnavailable):
    """The SolidWorks API type library could not be loaded for the installed
    version.  The version is auto-detected, but this build is only verified
    against SolidWorks 2024 -- a different release may need support added."""


def _require_typelibs():
    """``ensure_typelibs()`` but raise a CLEAR, actionable error if the typelib
    for the installed SolidWorks version cannot be loaded (instead of letting a
    cryptic [out]-parameter / OpenDoc6 failure surface much later)."""
    mod = ensure_typelibs()
    if mod is None and _HAVE_WIN32:
        detected = _registered_typelib_versions(_TYPELIBS["sldworks"])
        if detected:
            maj = detected[0][0]
            ver = f"typelib major {maj} (~SolidWorks {maj + 1992})"
        else:
            ver = "an unknown version (no SolidWorks typelib found in the registry)"
        raise SolidWorksTypelibError(
            f"Could not load the SolidWorks API type library for {ver}. This "
            f"build is verified with SolidWorks {_TESTED_MAJOR + 1992} (typelib "
            f"major {_TESTED_MAJOR}); your installed version may need support "
            f"added. Please report your SolidWorks version to the repository "
            f"maintainer: {_ISSUES_URL}")
    return mod


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
            _require_typelibs()
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
            self._sibling_dirs = {}      # src dir -> temp dir holding its siblings
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
        _require_typelibs()  # load typelibs first so Dispatch returns typed
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
        self._sibling_dirs = {}          # src dir -> temp dir holding its siblings
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
        import time as _time
        t0 = _time.time()
        doc = None
        # If the SAME assembly is already open in the user's reused session, use
        # THAT document: it is already fully resolved (the user can see it), so
        # we extract exactly what SolidWorks shows instead of a fragile copy +
        # reference re-resolution that can come up with 0 components for a
        # .SLDASM whose parts live elsewhere.  Read-only: extraction never saves,
        # and attach mode never closes the user's docs.
        if getattr(self, "_attached", False):
            existing = self._find_open_doc(path)
            if existing is not None:
                print("      reusing the already-open document from your "
                      "SolidWorks session (references already resolved)")
                self._reused_open_doc = True
                doc = existing
        t1 = t0
        if doc is None:
            # A temp copy loses SolidWorks' "same folder as the assembly"
            # reference-resolution rule; assemblies whose STORED part paths are
            # stale then open with almost everything unresolved (home_drone came
            # up as a single component).  Register the original's folder family
            # as document search paths so rule (2) replaces rule (4).
            # resolve the assembly's references to their CURRENT locations (also
            # recovers a part that was MOVED so the stored path is stale), then
            # both register those folders and co-locate the parts beside the copy
            dep_files = (self._dependency_files(path)
                         if doc_type_for(path) == SW_DOC_ASSEMBLY else [])
            self._register_search_folders(path, dep_files)
            # Reuse ONE temp dir per source folder: assemblies that share a folder
            # (the common case -- a module's sub-assemblies all sit beside it)
            # then co-locate their sibling CAD files ONCE instead of re-copying
            # the whole folder (38 files here) on every open_copy.
            src_dir = os.path.dirname(path)
            tmpdir = self._sibling_dirs.get(src_dir)
            first_for_dir = tmpdir is None
            if first_for_dir:
                tmpdir = tempfile.mkdtemp(prefix="sw2urdf_")
                self._tempdirs.append(tmpdir)
                self._sibling_dirs[src_dir] = tmpdir
            # Open the copy under a UNIQUE title so it never clashes with the
            # SAME assembly already open in the reused session -- OpenDoc6
            # otherwise fails with swFileWithSameTitleAlreadyOpen (65536).
            # Referenced parts are resolved by their own names (co-located
            # below), so renaming the top assembly is safe.
            stem, ext = os.path.splitext(os.path.basename(path))
            tmp = os.path.join(tmpdir, f"{stem}_{os.path.basename(tmpdir)}{ext}")
            if not os.path.exists(tmp):
                shutil.copy2(path, tmp)
            # Co-locate the source folder's sibling CAD files in the SAME temp dir
            # so SolidWorks' "same folder as the assembly" rule resolves them by
            # name -- ONCE per folder (the folder's other sub-assemblies reuse
            # this same temp dir).  The search-folder registration above does NOT
            # always override the absolute paths a downloaded (Pack-and-Go)
            # assembly bakes in -- those then open with every component unresolved
            # even though the parts sit right next to the .SLDASM.  A flat copy
            # fixes that; deep sub-folder layouts fall back to the search folders.
            if first_for_dir and doc_type_for(path) == SW_DOC_ASSEMBLY:
                try:
                    siblings = os.listdir(src_dir)
                except OSError:
                    siblings = []
                n_sib = 0
                for fn in siblings:
                    if fn.startswith("~$") or not fn.lower().endswith(
                            (".sldprt", ".sldasm", ".slddrw")):
                        continue                # lock files / non-CAD
                    s = os.path.join(src_dir, fn)
                    d = os.path.join(tmpdir, fn)
                    if os.path.isfile(s) and not os.path.exists(d):
                        try:
                            shutil.copy2(s, d)
                            n_sib += 1
                        except OSError:
                            pass
                if n_sib:
                    print(f"      co-located {n_sib} sibling CAD file(s) for "
                          f"reference resolution")
            # Also co-locate this assembly's OWN referenced files, resolved to
            # their current locations, beside the copy.  This is what recovers a
            # DISTANT shared library or a MOVED part whose stored path is stale:
            # a temp copy does not re-resolve an unresolvable stored path through
            # the search paths (those components then come up suppressed), but the
            # "same folder as the assembly" rule resolves a co-located part by name.
            n_dep = 0
            for s in dep_files:
                d = os.path.join(tmpdir, os.path.basename(s))
                if os.path.isfile(s) and not os.path.exists(d):
                    try:
                        shutil.copy2(s, d)
                        n_dep += 1
                    except OSError:
                        pass
            if n_dep:
                print(f"      co-located {n_dep} referenced file(s) from their "
                      f"resolved locations")
            t1 = _time.time()
            doc, ecode, wcode = open_doc6(self.app, tmp, doc_type_for(tmp))
            if doc is None:
                # 65536 = swFileWithSameTitleAlreadyOpen: a doc with this title
                # is already open in the reused session (we copy under a unique
                # title, so rare -- but a referenced part sharing a title with
                # the user's open work can still trip it)
                if ecode == 65536:
                    raise RuntimeError(
                        "SolidWorks refused to open the assembly because a "
                        "document with the same name is already open in your "
                        "running SolidWorks (error 65536). Close that document "
                        "in SolidWorks (or close SolidWorks entirely so sw2robot "
                        "opens it in a clean hidden instance), then retry.")
                raise RuntimeError(
                    f"OpenDoc6 failed for {tmp} "
                    f"(errors={ecode}, warnings={wcode})")
            self._open_docs.append(doc)
        t2 = _time.time()
        t3 = t2
        if doc_type_for(path) == SW_DOC_ASSEMBLY:
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

    def _dependency_files(self, src_path):
        """Every file this assembly references, resolved to its CURRENT location
        via ``GetDocumentDependencies2`` (from the file, no open required).

        SearchSubfolders is ON, which is what recovers a MOVED or relocated part:
        an assembly saved on another machine bakes in a stale absolute path (a
        different Drive letter, a ``\\.shortcut-targets-by-id\\`` id, an old
        folder), so the recorded path no longer exists here.  With the flag on,
        SolidWorks searches and returns the part's real, current location.  We use
        these both to seed the reference search paths AND to co-locate the parts
        beside the throw-away assembly copy (a distant SHARED library -- e.g.
        ``.../common_parts/servo/...`` while the assembly is under
        ``.../beetle/asm/subasm`` -- is otherwise never reached, and the assembly
        opens with every component unresolved: "no usable components")."""
        files = []
        try:
            # (document, Traverseflag=whole tree, SearchSubfolders=ON, AddReadOnlyInfo=off)
            deps = self.app.GetDocumentDependencies2(src_path, True, True, False)
        except Exception as e:
            print(f"      WARN: GetDocumentDependencies2 failed ({e!r}); "
                  f"distant/relocated part references may not resolve")
            return files
        if not deps:
            return files
        # flat [name, path, name, path, ...]; keep the existing resolved files
        seen = set()
        for p in list(deps)[1::2]:
            try:
                ap = os.path.abspath(str(p))
            except Exception:
                continue
            key = os.path.normcase(ap)
            if key not in seen and os.path.isfile(ap):
                seen.add(key)
                files.append(ap)
        return files

    def _register_search_folders(self, src_path, dep_files=()):
        """Add the source assembly's folder, its parent, the parent's sub-folders
        (parts/, commercial/, ...) AND every folder that actually holds a
        referenced part (from ``dep_files``) to the reference search paths of this
        SolidWorks session.

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
            # the real folders of every referenced part, wherever they live --
            # this is what lets a DISTANT shared library resolve (parent-subfolder
            # scan above only reaches parts near the .SLDASM)
            ref_dirs = []
            for f in dep_files:
                d = os.path.dirname(f)
                if d and d not in cand and d not in ref_dirs:
                    ref_dirs.append(d)
            cand.extend(ref_dirs)
            # KEEP the user's existing referenced-document search paths and only
            # ADD ours -- SolidWorks resolves this assembly's parts through those
            # configured paths interactively, so replacing them (the old
            # behaviour) made everything open unresolved when the .SLDASM sits
            # somewhere its parts are not (e.g. a lone copy in Downloads).
            prev = ""
            if getattr(self, "_saved_search_prefs", None):
                prev = self._saved_search_prefs[0] or ""
            prev_list = [p for p in str(prev).split(";") if p]
            merged = list(dict.fromkeys(cand + prev_list))
            folders = ";".join(merged)
            try:
                self.app.SetUserPreferenceToggle(
                    constants.swUseFolderSearchRule, True)
            except Exception:
                pass
            self.app.SetSearchFolders(constants.swDocumentType, folders)
            print(f"      reference search folders set ({len(cand)} dirs "
                  f"around {os.path.basename(src_dir)}, "
                  f"{len(ref_dirs)} from dependency records + "
                  f"{len(prev_list)} existing)")
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

    def _find_open_doc(self, path):
        """An already-open ModelDoc2 whose title matches ``path``'s file name, or
        None.  Lets a reused session extract from the user's resolved document
        instead of opening a (possibly unresolvable) copy."""
        want = os.path.splitext(os.path.basename(path))[0].lower()
        docs = safe_call(self.app, "GetDocuments")
        if docs is None:
            # fall back to the linked-list walk on versions without GetDocuments
            docs = []
            d = safe_call(self.app, "GetFirstDocument")
            seen = 0
            while d is not None and seen < 1000:
                docs.append(d)
                d = safe_call(d, "GetNext")
                seen += 1
        for d in (docs or []):
            try:
                title = safe_prop(d, "GetTitle") or ""
            except Exception:
                continue
            if os.path.splitext(title)[0].lower() == want:
                return d
        return None

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
