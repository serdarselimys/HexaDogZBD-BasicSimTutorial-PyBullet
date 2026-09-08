import sys
from pathlib import Path
from unittest.mock import MagicMock

INPUT_FILE  = "HexaDog_ZBD.xacro"   # xacro file in this folder
OUTPUT_FILE = "HexaDog_ZBD.urdf"    # urdf file to write into this folder


URDF_DIR = Path(__file__).resolve().parent      # the urdf folder
PACKAGE_ROOT = URDF_DIR.parent                  # the package folder above it

# Pretend the ROS 2 package index exists, so $(find <pkg>) resolves to the package folder
_mock_ament = MagicMock()
_mock_ament.get_package_share_directory.return_value = str(PACKAGE_ROOT)
sys.modules["ament_index_python"] = _mock_ament
sys.modules["ament_index_python.packages"] = _mock_ament

try:
    import xacro
except ImportError:
    sys.exit("xacro is not installed. Install it with:  pip install xacro")


def main():
    input_path = URDF_DIR / INPUT_FILE
    output_path = URDF_DIR / OUTPUT_FILE

    if not input_path.is_file():
        found = sorted(p.name for p in URDF_DIR.glob("*.xacro"))
        msg = f"Could not find '{INPUT_FILE}' in {URDF_DIR}"
        if found:
            msg += "\n\n.xacro files in this folder:\n  " + "\n  ".join(found)
        else:
            msg += "\n\nNo .xacro files here. Is the script in the right folder?"
        sys.exit(msg)

    try:
        doc = xacro.process_file(str(input_path))
        output_path.write_text(doc.toprettyxml(indent="  "))
    except Exception as e:
        sys.exit(f"Conversion failed: {e}")

    print("--- SUCCESS ---")
    print(f"Generated URDF at: {output_path}")


if __name__ == "__main__":
    main()
