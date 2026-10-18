import unittest

from modules.lora_route import RouteAssembler
from utils.nmea_packet import decode


def rte(line: str) -> tuple[str, ...]:
    packet = decode(line)
    assert packet is not None and packet.command == "RTE", line
    return packet.fields


def with_cs(body: str) -> str:
    value = 0
    for char in body:
        value ^= ord(char)
    return f"${body}*{value:02X}\n"


class RouteAssemblerTest(unittest.TestCase):
    def test_lines_in_any_order_make_the_route_once(self):
        assembler = RouteAssembler()
        second = rte(with_cs("RTE,12,7,2,3,1,10.883546;106.796248"))
        first = rte(with_cs("RTE,11,7,0,3,1,10.883438;106.796110,10.883438;-106.796248"))
        self.assertIsNone(assembler.feed(second))
        route = assembler.feed(first)
        self.assertIsNotNone(route)
        self.assertEqual(route.rev, 7)
        self.assertTrue(route.loop)
        self.assertEqual(route.points, ((10.883438, 106.796110), (10.883438, -106.796248), (10.883546, 106.796248)))
        # The board resends until $HBT reports rev 7: no second hand-over.
        self.assertIsNone(assembler.feed(first))
        self.assertIsNone(assembler.feed(second))

    def test_a_new_revision_replaces_a_half_received_one(self):
        assembler = RouteAssembler()
        self.assertIsNone(assembler.feed(("1", "3", "0", "2", "0", "10.0;106.0")))
        route = assembler.feed(("2", "4", "0", "1", "0", "11.0;107.0"))
        self.assertEqual((route.rev, route.points), (4, ((11.0, 107.0),)))
        self.assertIsNone(assembler.feed(("3", "3", "1", "2", "0", "10.5;106.5")))   # rev 3 starts over

    def test_an_empty_route_clears(self):
        route = RouteAssembler().feed(("1", "9", "0", "0", "0"))
        self.assertEqual((route.rev, route.points), (9, ()))

    def test_malformed_lines_are_refused(self):
        for fields in (("1", "0", "0", "1", "0", "1;2"), ("1", "5", "0", "301", "0"), ("1", "5", "0", "1", "2", "1;2"),
                       ("1", "5", "0", "1", "0", "95;2"), ("1", "5", "0", "1", "0", "1;2", "3;4"), ("1", "5", "0", "1", "0", "1,2"),
                       ("1", "5", "0")):
            with self.assertRaises(ValueError, msg=fields):
                RouteAssembler().feed(fields)


if __name__ == "__main__":
    unittest.main()
