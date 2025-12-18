#!/usr/bin/env python3
"""
Script to automatically update function links in TEST_SPECS.md reference table.

This script:
1. Parses TEST_SPECS.md to find all function links in the reference table
2. Scans Python test files to find actual function definitions and line numbers
3. Updates the links with correct relative paths and line numbers

Usage:
    python tests/update_func_links.py
"""

import os
import re
import ast
from pathlib import Path


def find_function_definition(file_path, function_name):
    """
    Find the line number where a function is defined in a Python file.
    
    Args:
        file_path (str): Path to the Python file
        function_name (str): Name of the function to find
        
    Returns:
        int or None: Line number where function is defined, or None if not found
    """
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # Parse the Python file into an AST
        tree = ast.parse(content)
        
        # Walk through all nodes in the AST
        for node in ast.walk(tree):
            if isinstance(node, ast.FunctionDef) and node.name == function_name:
                return node.lineno
                
    except (SyntaxError, FileNotFoundError, UnicodeDecodeError) as e:
        print(f"Error parsing {file_path}: {e}")
        
    return None


def scan_test_files(test_dirs):
    """
    Recursively scan all Python test files and build a mapping of test function names to their locations.
    
    Args:
        test_dirs (list): List of directories to scan recursively for test files
        
    Returns:
        dict: Mapping of function_name -> (file_path, line_number)
    """
    function_map = {}
    
    for test_dir in test_dirs:
        if not os.path.exists(test_dir):
            print(f"Warning: Directory {test_dir} does not exist")
            continue
            
        print(f"Scanning directory: {test_dir}")
        
        # Recursively walk through all subdirectories
        for root, dirs, files in os.walk(test_dir):
            for file in files:
                # Only check files that start with "test_" and end with ".py"
                if file.startswith('test_') and file.endswith('.py'):
                    file_path = os.path.join(root, file)
                    print(f"  Scanning file: {file_path}")
                    
                    try:
                        with open(file_path, 'r', encoding='utf-8') as f:
                            content = f.read()
                        
                        # Parse the Python file
                        tree = ast.parse(content)
                        
                        # Count all function definitions for debugging
                        all_functions = 0
                        test_functions_found = 0
                        
                        # Find all function definitions that start with "test_"
                        # This includes both top-level functions and class methods
                        for node in ast.walk(tree):
                            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
                                all_functions += 1
                                if node.name.startswith('test_'):
                                    function_map[node.name] = (file_path, node.lineno)
                                    test_functions_found += 1
                                    print(f"    Found test function: {node.name} at line {node.lineno}")
                        
                        print(f"    Total functions: {all_functions}, Test functions: {test_functions_found}")
                                
                    except (SyntaxError, FileNotFoundError, UnicodeDecodeError) as e:
                        print(f"Warning: Could not parse {file_path}: {e}")
                        continue
    
    return function_map


def get_relative_path(from_file, to_file):
    """
    Get the relative path from one file to another.
    
    Args:
        from_file (str): Source file path
        to_file (str): Target file path
        
    Returns:
        str: Relative path from source to target
    """
    from_path = Path(from_file).parent
    to_path = Path(to_file)
    
    try:
        return os.path.relpath(to_path, from_path)
    except ValueError:
        # If relative path can't be computed, return absolute path
        return str(to_path)


def update_test_specs_links(specs_file, function_map):
    """
    Update all function links in the TEST_SPECS.md file.
    
    Args:
        specs_file (str): Path to TEST_SPECS.md
        function_map (dict): Mapping of function names to their locations
    """
    if not os.path.exists(specs_file):
        print(f"Error: {specs_file} not found")
        return
    
    # Read the file
    with open(specs_file, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Pattern to match function links in the reference table
    # Matches: [function_name](path#Lnumber)
    pattern = r'\[([a-zA-Z_][a-zA-Z0-9_]*)\]\([^)]+#L\d+\)'
    
    def replace_link(match):
        function_name = match.group(1)
        
        if function_name in function_map:
            file_path, line_num = function_map[function_name]
            relative_path = get_relative_path(specs_file, file_path)
            
            # Construct the new link
            new_link = f"[{function_name}]({relative_path}#L{line_num})"
            
            print(f"Updated: {function_name} -> {relative_path}#L{line_num}")
            return new_link
        else:
            print(f"Warning: Function '{function_name}' not found in test files")
            return match.group(0)  # Return original if not found
    
    # Replace all matching links
    original_content = content
    updated_content = re.sub(pattern, replace_link, content)
    
    # Write back to file if changes were made
    if updated_content != original_content:
        with open(specs_file, 'w', encoding='utf-8') as f:
            f.write(updated_content)
        print(f"\nUpdated {specs_file}")
        
        # Count how many links were updated
        original_links = len(re.findall(pattern, original_content))
        updated_links = len(re.findall(pattern, updated_content))
        print(f"Processed {original_links} function links")
        
    else:
        print("No changes needed")


def main():
    """Main function to update all function links."""

    specs_file = os.path.join('.', 'TEST_SPECS.md')
    test_dirs = [Path('.')]
    
    print("Scanning test files for function definitions...")
    function_map = scan_test_files(test_dirs)
    
    print(f"Found {len(function_map)} test functions")
    
    if function_map:
        print("\nUpdating function links in TEST_SPECS.md...")
        update_test_specs_links(specs_file, function_map)
    else:
        print("No test functions found to update")


if __name__ == "__main__":
    main()