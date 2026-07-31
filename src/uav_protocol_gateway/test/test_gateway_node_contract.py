#!/usr/bin/env python3
import ast
import unittest
from pathlib import Path


GATEWAY_NODE = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "uav_protocol_gateway"
    / "gateway_node.py"
)


class GatewayNodeContractTests(unittest.TestCase):
    def test_payload_speedup_event_uses_qos_one(self):
        text = GATEWAY_NODE.read_text(encoding="utf-8")
        tree = ast.parse(text)
        publish_event = next(
            node
            for node in ast.walk(tree)
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
            and node.name == "_publish_event"
        )
        source = ast.get_source_segment(text, publish_event)

        self.assertIn(
            'qos = 1 if event == "PAYLOAD_RELEASED_SPEED_UP" else 0',
            source,
        )
        self.assertIn(
            "self._safe_publish(Topics.UAV_EVENT, message, qos=qos)",
            source,
        )


if __name__ == "__main__":
    unittest.main()
