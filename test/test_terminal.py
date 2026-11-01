import io
import os
import unittest
from unittest.mock import patch

from so101_driver.terminal import TerminalReadings, fit_line


class TtyStream(io.StringIO):
    def isatty(self):
        return True


class TerminalTests(unittest.TestCase):
    def test_non_tty_does_not_append_periodic_tables(self):
        stream = io.StringIO()
        terminal = TerminalReadings(stream, interval=0)
        terminal.update(["shoulder_pan: 2000"])
        self.assertEqual(stream.getvalue(), "")

        def answer(prompt):
            terminal.update(["shoulder_pan: 2010"])
            terminal.update(["shoulder_pan: 2020"])
            return "save"

        with patch("builtins.input", side_effect=answer):
            self.assertEqual(terminal.read("記録結果\nsave> "), "save")
        output = stream.getvalue()
        self.assertIn("2000", output)
        self.assertNotIn("2010", output)
        self.assertNotIn("2020", output)
        self.assertNotIn("\033", output)
        self.assertIsNone(terminal.prompt)

    def test_tty_redraws_existing_block_and_keeps_typed_text(self):
        stream = TtyStream()
        terminal = TerminalReadings(stream, interval=0)
        terminal.update(["header", "shoulder_pan: 2000"])

        def answer(prompt):
            terminal.update(["header", "shoulder_pan: 2010"])
            terminal.update(["INVALID"])
            return "save"

        with (
            patch("shutil.get_terminal_size", return_value=os.terminal_size((80, 24))),
            patch("readline.get_line_buffer", return_value="sa"),
            patch("builtins.input", side_effect=answer),
        ):
            self.assertEqual(terminal.read("save> "), "save")
        output = stream.getvalue()
        self.assertEqual(output.count("\033[2A"), 2)
        self.assertEqual(output.count("\033[J"), 2)
        self.assertTrue(output.endswith("save> sa"))
        terminal.update(["ignored after input"])
        self.assertEqual(stream.getvalue(), output)

    def test_japanese_lines_fit_without_wrapping(self):
        self.assertEqual(fit_line("正常abc", 6), "正常ab")
        self.assertEqual(fit_line("正常abc", 3), "正")
