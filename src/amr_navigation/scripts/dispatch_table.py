#!/usr/bin/env python3
"""
Convenience standalone wrapper for AMR Table Dispatcher
Usage:
    python3 src/amr_navigation/scripts/dispatch_table.py <ID>
    python3 src/amr_navigation/scripts/dispatch_table.py --list
"""
import sys
import os

# Add package directory to path
script_dir = os.path.dirname(os.path.abspath(__file__))
pkg_dir = os.path.dirname(script_dir)
if pkg_dir not in sys.path:
    sys.path.insert(0, pkg_dir)

from amr_navigation.dispatch_table import main

if __name__ == '__main__':
    sys.exit(main())
