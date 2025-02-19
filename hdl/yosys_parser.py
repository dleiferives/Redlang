import json
import argparse

class YosysJSONParser:
    def __init__(self, json_file_path):
        """
        Initializes the YosysJSONParser by loading JSON data from a file.

        Args:
            json_file_path (str): The path to the Yosys JSON file.
        """
        try:
            with open(json_file_path, 'r') as f:
                self.json_data = json.load(f)
            self.modules = {}
            if "modules" in self.json_data:
                self._parse_modules()
        except FileNotFoundError:
            print(f"Error: File not found at '{json_file_path}'")
            self.json_data = None  # Or handle the error as needed
            self.modules = {}
        except json.JSONDecodeError:
            print(f"Error: Invalid JSON format in '{json_file_path}'")
            self.json_data = None
            self.modules = {}

    def _parse_modules(self):
        """
        Parses the 'modules' section of the JSON data.
        """
        for module_name, module_data in self.json_data["modules"].items():
            self.modules[module_name] = self._parse_module(module_data)

    def _parse_module(self, module_data):
        """
        Parses a single module.

        Args:
            module_data (dict): The data for a single module.

        Returns:
            dict: A dictionary representing the parsed module.
        """
        module = {
            "attributes": module_data.get("attributes", {}),
            "ports": {},
            "cells": {},
            "netnames": {}
        }

        if "ports" in module_data:
            for port_name, port_data in module_data["ports"].items():
                module["ports"][port_name] = port_data

        if "cells" in module_data:
            for cell_name, cell_data in module_data["cells"].items():
                module["cells"][cell_name] = cell_data

        if "netnames" in module_data:
            for netname, netname_data in module_data["netnames"].items():
                module["netnames"][netname] = netname_data

        return module

    def get_module(self, module_name):
        """
        Retrieves a specific module by name.

        Args:
            module_name (str): The name of the module to retrieve.

        Returns:
            dict: The parsed module data, or None if the module is not found.
        """
        return self.modules.get(module_name)

    def get_all_modules(self):
        """
        Retrieves all parsed modules.

        Returns:
            dict: A dictionary of all parsed modules, keyed by module name.
        """
        return self.modules

    def print_module_summary(self, module_name):
        """
        Prints a summary of a module's contents.

        Args:
            module_name (str): The name of the module to summarize.
        """
        module = self.get_module(module_name)
        if module:
            print(f"Module: {module_name}")
            print("  Ports:")
            for port_name, port_data in module["ports"].items():
                print(f"    {port_name}: {port_data}")
            print("  Cells:")
            for cell_name, cell_data in module["cells"].items():
                print(f"    {cell_name}: {cell_data.get('type', 'Unknown')}")
            print("  Netnames:")
            for netname, netname_data in module["netnames"].items():
                print(f"    {netname}: {netname_data}")
        else:
            print(f"Module '{module_name}' not found.")

# Example Usage
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Yosys JSON Parser")
    parser.add_argument("json_file", help="Path to the Yosys JSON file")
    args = parser.parse_args()

    yosys_parser = YosysJSONParser(args.json_file)

    if yosys_parser.json_data:
        # Accessing and printing module information
        yosys_parser.print_module_summary("counter")  # Assuming "counter" is a module in your file

        # Get all modules
        all_modules = yosys_parser.get_all_modules()
        print("\nAll Modules:")
        for module_name, module_data in all_modules.items():
            print(f"- {module_name}")

        # Get a specific module
        counter_module = yosys_parser.get_module("counter")
        if counter_module:
            print("\nCounter Module (raw data):")
            # You can now work with the counter_module dictionary
            # For example, print the cells:
            print("  Cells:")
            for cell_name, cell_data in counter_module["cells"].items():
                print(f"    {cell_name}: {cell_data.get('type', 'Unknown')}")
    else:
        print("Failed to load the Yosys JSON file.")
