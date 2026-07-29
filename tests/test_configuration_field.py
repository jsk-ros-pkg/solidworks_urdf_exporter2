"""Configuration-aware extraction: the SolidWorks configuration an instance
references is captured on Component/ComponentState and must survive the
graph.json round-trip (extract keys mesh/mass on (part_path, configuration);
losing it renders two length-configured instances as the same variant)."""
import json

import numpy as np
import pytest

from sw2robot.exporter.model import Component, _component_states, from_graph
from sw2robot.exporter.state import ComponentState, GraphState


def _component(name, cfg):
    return Component(name=name, link_name=name, part_path=f"{name}.SLDPRT",
                     is_subassembly=False, world=np.eye(4), fixed=False, dof=0,
                     configuration=cfg)


def _state(name, cfg):
    return ComponentState(name=name, link_name=name, part_path=f"{name}.SLDPRT",
                          world=[float(x) for x in np.eye(4).flatten()],
                          configuration=cfg)


def _graph(*states):
    return GraphState(robot_name="r", source_assembly="a.SLDASM",
                      components=list(states))


def test_component_states_carries_configuration():
    (cs,) = _component_states([_component("tube-1", "180cm_shoulder_L120")])
    assert cs.configuration == "180cm_shoulder_L120"


def test_configuration_survives_json_round_trip():
    g = _graph(_state("tube-1", "shoulder_L120"), _state("tube-2", "wrist_L75"))
    g2 = GraphState.model_validate_json(g.model_dump_json())
    assert [c.configuration for c in g2.components] == ["shoulder_L120",
                                                        "wrist_L75"]


def test_from_graph_reconstructs_configuration():
    comps, _adj, _ground = from_graph(_graph(_state("tube-1", "wrist_L75")))
    assert comps[0].configuration == "wrist_L75"


def test_configuration_defaults_none_on_older_extract():
    # a graph.json written before this field existed simply lacks the key
    d = json.loads(_graph(_state("x-1", "cfgA")).model_dump_json())
    del d["components"][0]["configuration"]
    g = GraphState.model_validate_json(json.dumps(d))
    assert g.components[0].configuration is None


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))
