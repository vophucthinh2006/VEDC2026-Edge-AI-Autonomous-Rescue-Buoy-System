import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
from test_prototype import ROOT, config
from main import Engine
from protocol import diag

@unittest.skipUnless(shutil.which("gcc"), "host gcc required for actual shore C integration")
class ShoreIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.exe = Path(cls.temp.name)/("shore.exe" if os.name == "nt" else "shore")
        shore = ROOT.parent/"esp32-lora-station/main"
        subprocess.run(["gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-I", str(shore),
                        str(ROOT/"tests/shore_fixture.c"), str(shore/"lora_payload.c"),
                        str(shore/"victim_claims.c"),str(shore/"route_downlink.c"),"-lm","-o",str(cls.exe)],check=True)

    @classmethod
    def tearDownClass(cls): cls.temp.cleanup()

    def run_case(self, existing):
        e=Engine(config(),lambda *a,**kw: None); e.detected(.85,1)
        frame,_=e.next_frame(1)
        reply=subprocess.check_output([str(self.exe),existing,frame],text=True).strip()
        e.receive(reply,2)
        self.assertIsNone(e.pending)
        return e

    def test_the_station_reads_the_diagnostics_frame(self):
        read = lambda fps: subprocess.check_output([str(self.exe), "diag", diag("PHAO-02", fps)], text=True).strip()
        self.assertEqual(read(2.34), "PHAO-02 stationary fps=23")
        self.assertEqual(read(None), "PHAO-02 stationary fps=-1")
        self.assertEqual(read(500), "PHAO-02 stationary fps=999")

    def test_phao1_already_owns_person(self):
        e=self.run_case("existing")
        self.assertEqual(e.state,"SUPPRESSED")
        e.detected(.9,30); self.assertIsNone(e.pending)

    def test_phao2_reports_first(self):
        self.assertEqual(self.run_case("new").state,"REPORT_ACKED")
