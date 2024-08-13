#!/usr/bin/env python
# encoding: utf-8

'''
Build SITL default parameter files for use with SITL launcher
'''

import os
import re
import fnmatch
import argparse


def main():
    parser = argparse.ArgumentParser(description='Build SITL default parameter files')
    parser.add_argument('input_file', help='Input file to process')
    parser.add_argument('output_file', help='Output file to write to')
    args = parser.parse_args()

    defaults = process_defaults(args.input_file)
    with open(args.output_file, "w") as f:
        f.write("\n".join(defaults))


def process_defaults(file, depth=0):
    if depth > 10:
        raise Exception("Too many levels of @include")

    param_list = []
    with open(file, "r") as f:
        lines = f.read().splitlines()
    for line in lines:
        if line.startswith("@include"):
            rel_path = line.split(maxsplit=1)[1]
            path = os.path.join(os.path.dirname(file), rel_path)
            param_list.extend(
                process_defaults(path, depth + 1)
            )
            continue

        if line.startswith("@delete"):
            pattern = line.split(maxsplit=1)[1]
            for i in range(len(param_list)):
                param_name = re.split(r"[\s,]+", param_list[i])[0]
                if fnmatch.fnmatch(param_name, pattern):
                    param_list[i] = "#deleted " + param_list[i]
            line = "#" + line

        param_list.append(line)

    return param_list


if __name__ == "__main__":
    main()
