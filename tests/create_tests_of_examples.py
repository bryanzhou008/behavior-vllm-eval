import importlib
import os
import pkgutil
import shutil
from string import Template

import igibson
from igibson import examples
from igibson.utils.assets_utils import download_assets

download_assets()


def main():
    examples_list = []
    for package in pkgutil.walk_packages(examples.__path__, examples.__name__ + "."):
        if (
            not package.ispkg
            and package.name[17:] != "example_selector"
            and "web_ui" not in package.name[17:]
            and "vr_" not in package.name[17:]
        ):  # Consider removing the last condition if we have runnable VR tests
            examples_list += [package.name[17:]]

    temp_folder_of_test = os.path.join("/", "tmp", "tests_of_examples")
    shutil.rmtree(temp_folder_of_test, ignore_errors=True)
    os.makedirs(temp_folder_of_test, exist_ok=True)

    for example in examples_list:
        template_file_name = os.path.join(igibson.__path__[0], "..", "tests", "test_of_example_template.txt")
        with open(template_file_name, "r") as f:
            substitutes = dict()
            substitutes["module"] = example
            name = example.rsplit(".", 1)[-1]
            substitutes["name"] = name
            src = Template(f.read())
            dst = src.substitute(substitutes)
            test_file = open(os.path.join(temp_folder_of_test, name + "_test.py"), "w")
            n = test_file.write(dst)
            test_file.close()


if __name__ == "__main__":
    main()
