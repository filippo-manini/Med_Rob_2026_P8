import yaml
import importlib
import os
import sys
import numpy as np  # Required for matrix handling

class Parser:
    def __init__(self, config_path):
        self.config_path = config_path
        self.components = {}

    def load_config(self):
        try:
            with open(self.config_path, 'r') as f:
                self.config = yaml.safe_load(f)
        except FileNotFoundError:
            raise FileNotFoundError(f"Config file not found: {self.config_path}")
        except yaml.YAMLError as e:
            raise ValueError(f"Failed to parse YAML: {e}")

    def convert_params(self, params):
        """ Convert list values to numpy arrays where applicable """
        for key, value in params.items():
            if isinstance(value, list):
                # Check if the list is meant to represent a matrix (length 9 for 3x3)
                if len(value) == 9:  # Case for 3x3 matrix
                    params[key] = np.array(value).reshape(3, 3)
                else:
                    params[key] = np.array(value)
            elif isinstance(value, str) and value.startswith("$"):
                # Resolve references like "$container" to previously created components
                ref_name = value[1:]
                if ref_name not in self.components:
                    raise ValueError(f"Component references unknown component '{ref_name}'")
                params[key] = self.components[ref_name]
        return params

    def initialize_component(self, module_name, class_name, params):
        try:
            module = importlib.import_module(module_name)
        except ImportError as e:
            raise ImportError(f"Failed to import module '{module_name}': {e}")

        try:
            cls = getattr(module, class_name)
        except AttributeError:
            raise AttributeError(f"Class '{class_name}' not found in module '{module_name}'")

        try:
            return cls(**params)
        except Exception as e:
            raise RuntimeError(f"Error instantiating '{class_name}' from '{module_name}' with params {params}: {e}")

    def parse(self):
        self.load_config()

        for comp_name, comp_cfg in self.config.items():
            module_name = comp_cfg.get("module")
            class_name = comp_cfg.get("class")
            params = comp_cfg.get("params", {})
            attributes = comp_cfg.get("attributes", {})

            if not module_name or not class_name:
                raise ValueError(f"Component '{comp_name}' is missing 'module' or 'class' fields.")

            # Convert params (handle references and list-to-array conversion)
            params = self.convert_params(params)

            # Instantiate the component
            instance = self.initialize_component(module_name, class_name, params)

            # Set additional attributes post-instantiation
            for attr_name, attr_value in attributes.items():
                try:
                    setattr(instance, attr_name, attr_value)
                except Exception as e:
                    raise RuntimeError(f"Failed to set attribute '{attr_name}' on component '{comp_name}': {e}")

            # If the component has a validate_and_finalize method, call it
            if hasattr(instance, "validate_and_finalize") and callable(getattr(instance, "validate_and_finalize")):
                try:
                    print(f"[Parser] Finalizing component '{comp_name}' via validate_and_finalize()...")
                    instance.validate_and_finalize()
                except Exception as e:
                    raise RuntimeError(f"Error in validate_and_finalize() for component '{comp_name}': {e}")

            # Store the component
            self.components[comp_name] = instance

    def get_component(self, name):
        return self.components.get(name)

    def get_all_components(self):
        return self.components
