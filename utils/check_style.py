# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import subprocess
import re
import os
import shutil
import argparse
from pathlib import Path
import multiprocessing
from functools import partial
import time
import sys

# Yapf requires python 3.6+
if sys.version_info < (3, 6):
    raise RuntimeError(
        "Requires Python 3.6+, currently using Python {}.{}.".format(
            sys.version_info.major, sys.version_info.minor))
# Check and import yapf
# > not found: throw exception
# > version mismatch: throw exception
try:
    import yapf
except ImportError:
    raise ImportError(
        "yapf not found. Install with `pip install yapf==0.43.0`.")
if yapf.__version__ != "0.43.0":
    raise RuntimeError(
        "yapf 0.43.0 required. Install with `pip install yapf==0.43.0`.")
print("Using yapf version {}".format(yapf.__version__))

PYTHON_FORMAT_DIRS = [
    "imu",
    "kfc",
    "utils",
]

# Note: also modify CPP_FORMAT_DIRS in check_cpp_style.cmake.
CPP_FORMAT_DIRS = [
    "imu",
    "kfc",
]


def _glob_files(directories, extensions):
    """
    Find files with certain extensions in directories recursively.

    Args:
        directories: list of directories, relative to the root repo directory.
        extensions: list of extensions, e.g. ["cpp", "h"].

    Return:
        List of file paths.
    """
    pwd = Path(os.path.dirname(os.path.abspath(__file__)))
    root_dir = pwd.parent

    file_paths = []
    for directory in directories:
        directory = root_dir / directory
        for extension in extensions:
            extension_regex = "*." + extension
            file_paths.extend(directory.rglob(extension_regex))
    file_paths = [str(file_path) for file_path in file_paths]
    file_paths = sorted(list(set(file_paths)))
    return file_paths


def _find_clang_format():
    """
    Find clang-format:
      - not found: throw exception
      - version mismatch: print warning
    """
    preferred_clang_format_name = "clang-format-10"
    preferred_version_major = 10
    clang_format_bin = shutil.which(preferred_clang_format_name)
    if clang_format_bin is None:
        clang_format_bin = shutil.which("clang-format")
    if clang_format_bin is None:
        raise RuntimeError("clang-format not found.")
    version_str = subprocess.check_output([clang_format_bin, "--version"
                                          ]).decode("utf-8").strip()
    try:
        m = re.match("^clang-format version ([0-9.]*).*$", version_str)
        if m:
            version_str = m.group(1)
            version_str_token = version_str.split(".")
            major = int(version_str_token[0])
            if major != preferred_version_major:
                print("Warning: {} required, but got {}.".format(
                    preferred_clang_format_name, version_str))
        else:
            raise
    except:
        print("Warning: failed to parse clang-format version {}, "
              "please ensure {} is used.".format(version_str,
                                                 preferred_clang_format_name))
    print("Using clang-format version {}.".format(version_str))

    return clang_format_bin


class CppFormatter:

    standard_header = """// ----------------------------------------------------------------------------
// Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
//
// All rights reserved.
// ----------------------------------------------------------------------------
"""

    def __init__(self, file_paths, clang_format_bin):
        self.file_paths = file_paths
        self.clang_format_bin = clang_format_bin

    @staticmethod
    def _check_style(file_path, clang_format_bin):
        """
        Returns (true, true) if (style, header) is valid.
        """

        with open(file_path, 'r') as f:
            if f.read().startswith(CppFormatter.standard_header):
                is_valid_header = True
            else:
                is_valid_header = False

            # We don't check the header for now.
            is_valid_header = True

        cmd = [
            clang_format_bin,
            "-style=file",
            "-output-replacements-xml",
            file_path,
        ]
        result = subprocess.check_output(cmd).decode("utf-8")
        if "<replacement " in result:
            is_valid_style = False
        else:
            is_valid_style = True
        return (is_valid_style, is_valid_header)

    @staticmethod
    def _apply_style(file_path, clang_format_bin):
        cmd = [
            clang_format_bin,
            "-style=file",
            "-i",
            file_path,
        ]
        subprocess.check_output(cmd)

    def run(self, apply, no_parallel, verbose):
        if apply:
            print("Applying C++/CUDA style...")
        else:
            print("Checking C++/CUDA style...")

        if verbose:
            print("To format:")
            for file_path in self.file_paths:
                print("> {}".format(file_path))

        start_time = time.time()
        if no_parallel:
            is_valid_files = map(
                partial(self._check_style,
                        clang_format_bin=self.clang_format_bin),
                self.file_paths)
        else:
            with multiprocessing.Pool(multiprocessing.cpu_count()) as pool:
                is_valid_files = pool.map(
                    partial(self._check_style,
                            clang_format_bin=self.clang_format_bin),
                    self.file_paths)

        changed_files = []
        wrong_header_files = []
        for is_valid, file_path in zip(is_valid_files, self.file_paths):
            is_valid_style = is_valid[0]
            is_valid_header = is_valid[1]
            if not is_valid_style:
                changed_files.append(file_path)
                if apply:
                    self._apply_style(file_path, self.clang_format_bin)
            if not is_valid_header:
                wrong_header_files.append(file_path)
        print("Formatting takes {:.2f}s".format(time.time() - start_time))

        return (changed_files, wrong_header_files)


class PythonFormatter:

    standard_header = """# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------
"""

    def __init__(self, file_paths, style_config):
        self.file_paths = file_paths
        self.style_config = style_config

    @staticmethod
    def _check_style(file_path, style_config):
        """
        Returns (true, true) if (style, header) is valid.
        """

        with open(file_path, 'r') as f:
            content = f.read()
            if len(content) == 0 or content.startswith(
                    PythonFormatter.standard_header):
                is_valid_header = True
            else:
                is_valid_header = False

            # We don't check the header for now.
            is_valid_header = True

        _, _, changed = yapf.yapflib.yapf_api.FormatFile(
            file_path, style_config=style_config, in_place=False)
        return (not changed, is_valid_header)

    @staticmethod
    def _apply_style(file_path, style_config):
        _, _, _ = yapf.yapflib.yapf_api.FormatFile(file_path,
                                                   style_config=style_config,
                                                   in_place=True)

    def run(self, apply, no_parallel, verbose):
        if apply:
            print("Applying Python style...")
        else:
            print("Checking Python style...")

        if verbose:
            print("To format:")
            for file_path in self.file_paths:
                print("> {}".format(file_path))

        start_time = time.time()
        if no_parallel:
            is_valid_files = map(
                partial(self._check_style, style_config=self.style_config),
                self.file_paths)
        else:
            with multiprocessing.Pool(multiprocessing.cpu_count()) as pool:
                is_valid_files = pool.map(
                    partial(self._check_style, style_config=self.style_config),
                    self.file_paths)

        changed_files = []
        wrong_header_files = []
        for is_valid, file_path in zip(is_valid_files, self.file_paths):
            is_valid_style = is_valid[0]
            is_valid_header = is_valid[1]
            if not is_valid_style:
                changed_files.append(file_path)
                if apply:
                    self._apply_style(file_path, self.style_config)
            if not is_valid_header:
                wrong_header_files.append(file_path)

        print("Formatting takes {:.2f}s".format(time.time() - start_time))
        return (changed_files, wrong_header_files)


def _filter_files(files, ignored_patterns):
    return [
        file for file in files if not any(
            [ignored_pattern in file for ignored_pattern in ignored_patterns])
    ]


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--apply",
        dest="apply",
        action="store_true",
        default=False,
        help="Apply style to files in-place.",
    )
    parser.add_argument(
        "--no_parallel",
        dest="no_parallel",
        action="store_true",
        default=False,
        help="Disable parallel execution.",
    )
    parser.add_argument(
        "--verbose",
        dest="verbose",
        action="store_true",
        default=False,
        help="If true, prints file names while formatting.",
    )
    args = parser.parse_args()

    # Check formatting libs
    clang_format_bin = _find_clang_format()
    pwd = Path(os.path.dirname(os.path.abspath(__file__)))
    python_style_config = str(pwd.parent / ".style.yapf")

    cpp_ignored_files = []
    cpp_files = _glob_files(
        CPP_FORMAT_DIRS,
        ["h", "hpp", "cpp", "cuh", "cu", "isph", "ispc", "h.in"])
    cpp_files = _filter_files(cpp_files, cpp_ignored_files)

    # Check or apply style
    cpp_formatter = CppFormatter(cpp_files, clang_format_bin=clang_format_bin)
    python_formatter = PythonFormatter(_glob_files(PYTHON_FORMAT_DIRS, ["py"]),
                                       style_config=python_style_config)

    changed_files = []
    wrong_header_files = []
    changed_files_cpp, wrong_header_files_cpp = cpp_formatter.run(
        apply=args.apply, no_parallel=args.no_parallel, verbose=args.verbose)
    changed_files.extend(changed_files_cpp)
    wrong_header_files.extend(wrong_header_files_cpp)
    print(f"apply cpp file num is {len(changed_files_cpp)}")

    changed_files_python, wrong_header_files_python = python_formatter.run(
        apply=args.apply, no_parallel=args.no_parallel, verbose=args.verbose)
    changed_files.extend(changed_files_python)
    wrong_header_files.extend(wrong_header_files_python)
    print(f"apply python file num is {len(changed_files_python)}")

    if len(changed_files) == 0 and len(wrong_header_files) == 0:
        print("All files passed style check.")
        exit(0)

    if args.apply:
        if len(changed_files) != 0:
            print("Style applied to the following files:")
            print("\n".join(changed_files))
            exit(1)
        if len(wrong_header_files) != 0:
            print("Please correct license header *manually* in the following "
                  "files (see util/check_style.py for the standard header):")
            print("\n".join(wrong_header_files))
            exit(1)
    else:
        error_files_no_duplicates = list(set(changed_files +
                                             wrong_header_files))
        if len(error_files_no_duplicates) != 0:
            print("Style error found in the following files:")
            print("\n".join(error_files_no_duplicates))
            exit(1)
