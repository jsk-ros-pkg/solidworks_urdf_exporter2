"""Import the configuration the classic SW2URDF add-in left in the document.

An assembly configured with ``ros/solidworks_urdf_exporter`` carries its whole
URDF configuration inside the SolidWorks file: an attribute feature holding
the add-in's serialized link/joint tree, plus the reference geometry (CoordSys
link frames, RefAxis joint axes) that tree points at.  This package rebuilds
that configuration so sw2robot can export the URDF the author intended instead
of inferring one from mates.

Three routes, most to least authoritative -- the caller tries them in order
and each says loudly why it declined:

``payload``
    :func:`parse_sw2urdf_payload` + :func:`reconstruct_sw2urdf_config_from_payload`
    read the add-in's own serialized tree.  Nothing is inferred.
``naming``
    :func:`reconstruct_sw2urdf_config` rebuilds from the ``<link>_origin``
    dialog-default feature names.
``geometric``
    :func:`reconstruct_sw2urdf_config_geometric` matches reference axes to
    mate edges, so no naming convention is needed.

Every route returns the same mapping: ``links`` (component -> link name,
authored frame, member components), ``root_link`` and ``joints``.
"""

from .bridge import reconstruct_sw2urdf_config_from_payload
from .geometric import reconstruct_sw2urdf_config_geometric
from .naming import detect_sw2urdf_reference_geometry, reconstruct_sw2urdf_config
from .payload import parse_sw2urdf_payload

__all__ = [
    "detect_sw2urdf_reference_geometry",
    "parse_sw2urdf_payload",
    "reconstruct_sw2urdf_config",
    "reconstruct_sw2urdf_config_from_payload",
    "reconstruct_sw2urdf_config_geometric",
]
