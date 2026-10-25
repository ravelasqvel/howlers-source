"""
Omnidirectional Robot System Identification - Main Entry Point
===============================================================

Professional modular framework for three-stage parameter identification.

Usage:
    python main.py    # Run all examples with synthetic data

Author: Robot Identification Framework
Version: 2.0.0 (Modular)
"""

import sys
from src.examples import run_all_examples


def main():
    """Main entry point for robot identification examples."""
    try:
        success = run_all_examples()
        return 0 if success else 1
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        return 130
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
