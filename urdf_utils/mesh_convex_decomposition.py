#!/usr/bin/env python3
"""
Batch processing script for CoACD mesh decomposition.
This script processes all STL files in an input directory using CoACD
and saves the results to an output directory.

Usage examples:
1. Command line usage:
   python batch_coacd.py

2. As a module import:
   from batch_coacd import batch_convex_decomposition

   # Basic usage
   result = batch_convex_decomposition("/path/to/input/meshes", "/path/to/output/meshes")

   # Silent processing
   result = batch_convex_decomposition("/path/to/input", "/path/to/output", verbose=False)

   # Check results
   if result['failed'] == 0:
       print(f"All {result['successful']} files processed successfully!")
   else:
       print(f"Failed to process {result['failed']} files: {result['failed_files']}")
"""

import open3d as o3d
import numpy as np
import sys
from pathlib import Path
import coacd

# CoACD parameters (optimized for your use case)
COACD_PARAMS = {
    "threshold": 0.6,
    "max_convex_hull": 5,
    "preprocess_mode": "auto",
    "preprocess_resolution": 50,
    "resolution": 1000,
    "mcts_nodes": 20,
    "mcts_iterations": 150,
    "mcts_max_depth": 3,
    "pca": False,
    "merge": True,
    "decimate": True,
    "max_ch_vertex": 64,
    "extrude": False,
    "extrude_margin": 0.01,
    "apx_mode": "ch",
    "seed": 0,
}


def find_stl_files(input_dir):
    """Find all STL files in the input directory (case insensitive)."""
    input_path = Path(input_dir)
    stl_files = []

    # Search for STL files with both .stl and .STL extensions
    for pattern in ["*.stl", "*.STL"]:
        stl_files.extend(input_path.glob(pattern))

    return sorted(stl_files)


def run_coacd_on_file(input_file, output_file):
    """Run CoACD on a single file with the specified parameters."""
    try:
        print(f"Processing: {input_file.name} -> {output_file.name}")

        # Load mesh using open3d
        mesh = o3d.io.read_triangle_mesh(str(input_file))

        # Convert to numpy arrays
        vertices = np.asarray(mesh.vertices)
        triangles = np.asarray(mesh.triangles)

        # Convert to CoACD mesh format
        coacd_mesh = coacd.Mesh(vertices, triangles)

        # Run CoACD decomposition
        result = coacd.run_coacd(coacd_mesh, **COACD_PARAMS)

        # Convert result to open3d meshes and combine them
        combined_vertices = []
        combined_triangles = []
        vertex_offset = 0

        np.random.seed(0)
        for i, (vs, fs) in enumerate(result):
            # Add vertices with offset
            combined_vertices.extend(vs)

            # Add triangles with vertex offset
            triangles_with_offset = fs + vertex_offset
            combined_triangles.extend(triangles_with_offset)

            # Update vertex offset for next mesh
            vertex_offset += len(vs)

        # Create combined open3d mesh
        if combined_vertices and combined_triangles:
            combined_mesh = o3d.geometry.TriangleMesh()
            combined_mesh.vertices = o3d.utility.Vector3dVector(
                np.array(combined_vertices)
            )
            combined_mesh.triangles = o3d.utility.Vector3iVector(
                np.array(combined_triangles)
            )

            # Compute normals for better visualization
            combined_mesh.compute_vertex_normals()

            # Export the result
            o3d.io.write_triangle_mesh(str(output_file), combined_mesh)

        print(
            f"✓ Successfully processed {input_file.name} ({len(result)} convex hulls)"
        )
        return True

    except Exception as e:
        print(f"✗ Error processing {input_file.name}: {e}")
        return False


def batch_convex_decomposition(input_dir_path, output_dir_path, verbose=False):
    """
    Batch process STL files for convex decomposition using CoACD.

    Args:
        input_dir_path (str or Path): Path to the directory containing STL files
        output_dir_path (str or Path): Path to the output directory for processed files
        verbose (bool): Whether to print detailed progress information

    Returns:
        dict: Dictionary containing processing results with keys:
            - 'total_files': Total number of STL files found
            - 'successful': Number of successfully processed files
            - 'failed': Number of failed files
            - 'success_rate': Success rate as a percentage
            - 'failed_files': List of files that failed to process
    """
    # Set log level to error for cleaner output
    coacd.set_log_level("error")

    # Convert to Path objects
    input_dir = Path(input_dir_path)
    output_dir = Path(output_dir_path)

    if verbose:
        print("CoACD Batch Processing")
        print("=" * 50)

    # Validate input directory
    if not input_dir.exists():
        raise FileNotFoundError(f"Input directory '{input_dir}' does not exist")

    if not input_dir.is_dir():
        raise NotADirectoryError(f"'{input_dir}' is not a directory")

    # Create output directory if it doesn't exist
    output_dir.mkdir(parents=True, exist_ok=True)

    # Find all STL files
    stl_files = find_stl_files(input_dir)

    if not stl_files:
        if verbose:
            print(f"No STL files found in '{input_dir}'")
        return {
            "total_files": 0,
            "successful": 0,
            "failed": 0,
            "success_rate": 0.0,
            "failed_files": [],
        }

    if verbose:
        print(f"Input directory: '{input_dir}'")
        print(f"Output directory: '{output_dir}'")
        print(f"Found {len(stl_files)} STL file(s)")
        print(
            f"CoACD parameters: threshold={COACD_PARAMS['threshold']}, "
            f"max_convex_hull={COACD_PARAMS['max_convex_hull']}, "
            f"decimate={COACD_PARAMS['decimate']}, "
            f"max_ch_vertex={COACD_PARAMS['max_ch_vertex']}, "
            f"resolution={COACD_PARAMS['resolution']}"
        )
        print()

    # Process each STL file
    successful = 0
    failed = 0
    failed_files = []

    for stl_file in stl_files:
        # Generate output filename
        output_file = output_dir / f"{stl_file.stem}.STL"

        # Run CoACD on the file
        if run_coacd_on_file(stl_file, output_file):
            successful += 1
        else:
            failed += 1
            failed_files.append(str(stl_file))

    # Calculate success rate
    success_rate = (successful / len(stl_files) * 100) if stl_files else 0.0

    # Print summary if verbose
    if verbose:
        print()
        print("=" * 50)
        print("BATCH PROCESSING SUMMARY")
        print("=" * 50)
        print(f"Total files found: {len(stl_files)}")
        print(f"Successfully processed: {successful}")
        print(f"Failed: {failed}")
        print(f"Success rate: {success_rate:.1f}%")

        if failed > 0:
            print("\nSome files failed to process. Check the error messages above.")
        else:
            print("\nAll files processed successfully!")

    return {
        "total_files": len(stl_files),
        "successful": successful,
        "failed": failed,
        "success_rate": success_rate,
        "failed_files": failed_files,
    }


def main():
    """Main function for command-line usage."""
    # Configuration - Modify these paths as needed
    input_dir = "path_to_visual_mesh" 
    output_dir = "path_to_collision_mesh"

    try:
        result = batch_convex_decomposition(input_dir, output_dir, verbose=True)

        if result["failed"] > 0:
            sys.exit(1)
        else:
            sys.exit(0)

    except (FileNotFoundError, NotADirectoryError) as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
