import os
import parse
import sys

env = {}

def build_c(path):
    pass

def build_elf(all_paths):
    pass

all_files = {}
special_paths = []
builders = {}

builders[".c"] = build_c

def add_exclusive_path(path):
    special_paths.append(parse.compile("./" + path, case_sensitive=True))

add_exclusive_path("examples/{example_type}/{example_name}/")
add_exclusive_path("hw/bsp/{board}/")
add_exclusive_path("tests/{chip_family}/test")
add_exclusive_path("tests/{chip_family}/test/{test_name}")
add_exclusive_path("hw/mcu/{vendor}/{chip_family}/")
add_exclusive_path("src/portable/{vendor}/{chip_family}/")

def build_dir(path):
    path += os.sep
    for special_path in special_paths:
        match = special_path.search(path)
        if match:
            for k in match.named:
                if k not in env or match[k] not in env[k]:
                    #print("Skipping", path, "because of missing environment variable")
                    return
    with os.scandir(path) as it:
        for entry in it:
            if not entry.name.startswith('.'):
                fn = os.path.join(path, entry.name)
                if entry.is_file():
                    for extension in builders:
                        if fn.endswith(extension):
                            print(fn, )
                            continue
                    pass
                elif entry.is_dir():
                    build_dir(fn)

for arg in sys.argv[1:]:
    if not arg.startswith("--") or "=" not in arg:
        continue
    key, value = arg[2:].split("=", 1)
    env[key] = value.split(",")
build_dir(".")
