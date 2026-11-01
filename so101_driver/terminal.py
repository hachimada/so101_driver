"""Periodic calibration readings that keep the terminal input prompt visible."""

import readline
import shutil
import sys
import threading
import time
import unicodedata


def fit_line(text, width):
    """Avoid terminal wrapping, including wide Japanese characters."""
    result = []
    columns = 0
    for char in text:
        size = (
            0
            if unicodedata.combining(char)
            else (2 if unicodedata.east_asian_width(char) in ("W", "F") else 1)
        )
        if columns + size > width:
            break
        result.append(char)
        columns += size
    return "".join(result)


class TerminalReadings:
    def __init__(self, stream=None, interval=1.0):
        self.stream = stream if stream is not None else sys.stdout
        self.interval = interval
        self.lock = threading.RLock()
        self.lines = []
        self.prompt = None
        self.last_render = 0.0
        self.drawn_rows = 0
        self.columns = None

    def update(self, lines):
        with self.lock:
            self.lines = list(lines)
            if (
                self.stream.isatty()
                and self.prompt is not None
                and time.monotonic() - self.last_render >= self.interval
            ):
                self._render()

    def _display_lines(self, columns):
        return [
            fit_line(line, columns - 1) for line in "\n".join(self.lines).strip("\n").splitlines()
        ]

    def _render(self):
        typed = readline.get_line_buffer()
        columns = shutil.get_terminal_size().columns
        self.stream.write("\r")
        if self.columns == columns and self.drawn_rows:
            self.stream.write(f"\033[{self.drawn_rows}A")
        elif self.columns != columns:
            # Start a fresh block after terminal reflow instead of erasing unrelated text.
            self.stream.write("\n")
        self.stream.write("\033[J")
        lines = self._display_lines(columns)
        if lines:
            self.stream.write("\n".join(lines) + "\n")
        self.drawn_rows = len(lines)
        self.columns = columns
        self.stream.write(fit_line((self.prompt or "") + typed, columns - 1))
        self.stream.flush()
        self.last_render = time.monotonic()

    def read(self, text):
        prefix, _, last = text.rpartition("\n")
        with self.lock:
            if prefix:
                print(prefix, file=self.stream, flush=True)
            columns = shutil.get_terminal_size().columns
            if self.stream.isatty() and fit_line(last, columns - 16) != last:
                print(last, file=self.stream, flush=True)
                last = "> "
            self.prompt = last
            # input() prints the prompt itself; print the first snapshot separately.
            lines = self._display_lines(columns)
            if lines:
                self.stream.write("\n".join(lines) + "\n")
                self.stream.flush()
            self.drawn_rows = len(lines)
            self.columns = columns
            self.last_render = time.monotonic()
        try:
            return input(last)
        finally:
            with self.lock:
                self.prompt = None
