import os

def append_scala_files(directory_to_scan, output_file_handle, project_root_for_relative_paths):
    """
    Finds all Scala files in the directory_to_scan and its subdirectories,
    and appends their content to the already open output_file_handle.
    File paths in the Markdown are relative to project_root_for_relative_paths.
    """
    for root, _, files in os.walk(directory_to_scan):
        for file_name in files:
            if file_name.endswith(".scala"):
                file_path = os.path.join(root, file_name)
                # Generate a path relative to the overall project root for display
                display_relative_path = os.path.relpath(file_path, project_root_for_relative_paths)
                
                output_file_handle.write(f"## File: {display_relative_path}\n\n")
                output_file_handle.write("```scala\n")
                
                try:
                    with open(file_path, 'r', encoding='utf-8') as scala_file:
                        output_file_handle.write(scala_file.read())
                except Exception as e:
                    output_file_handle.write(f"// Error reading file {file_path}: {e}\n")
                
                output_file_handle.write("\n```\n\n")

def append_markdown_content(markdown_path_to_include, output_file_handle, project_root_for_relative_paths):
    """
    Appends the content of the specified markdown_path_to_include
    to the already open output_file_handle.
    The header for the included content uses a path relative to project_root_for_relative_paths.
    """
    try:
        # Generate a path relative to the overall project root for display
        display_relative_md_path = os.path.relpath(markdown_path_to_include, project_root_for_relative_paths)
        output_file_handle.write(f"## Content from: {display_relative_md_path}\n\n")
        with open(markdown_path_to_include, 'r', encoding='utf-8') as existing_md_file:
            output_file_handle.write(existing_md_file.read())
        output_file_handle.write("\n\n") # Add some spacing after included markdown
    except Exception as e:
        output_file_handle.write(f"// Error reading or appending markdown file {markdown_path_to_include}: {e}\n\n")

if __name__ == "__main__":
    # Define the base project directory.
    # This helps in constructing other paths and ensuring consistent relative paths in the output.
    base_project_dir = "/home/lzy/chipyard/generators/mycnnaccelerators"

    # Define Scala source directories
    scala_source_main_dir = os.path.join(base_project_dir, "src/main/scala/mycnnaccelerators")
    scala_source_test_dir = os.path.join(base_project_dir, "src/test/scala/mycnnaccelerators") # Additional Scala directory

    # Define the Markdown file whose content needs to be included
    markdown_to_include_path = os.path.join(base_project_dir, "RISC-V.md") # Markdown file to add

    # Define the output Markdown file (using the path from your original script)
    # This path is /home/lzy/chipyard/generators/mycnnaccelerators/src/main/scala/mycnnaccelerators/mycnnaccelerators_scala_files.md
    output_md_file_path = os.path.join(scala_source_main_dir, "mycnnaccelerators_scala_files.md")

    # Ensure the output directory exists
    output_md_directory = os.path.dirname(output_md_file_path)
    if not os.path.exists(output_md_directory):
        try:
            # exist_ok=True prevents an error if the directory already exists
            os.makedirs(output_md_directory, exist_ok=True)
            print(f"Created output directory: {output_md_directory}")
        except OSError as e:
            print(f"Error: Could not create output directory '{output_md_directory}'. {e}")
            exit(1) # Exit if we can't create the output directory

    sources_processed_count = 0

    try:
        with open(output_md_file_path, 'w', encoding='utf-8') as md_file:
            # Process Scala files from the main source directory
            if os.path.isdir(scala_source_main_dir):
                print(f"Processing Scala files from: {scala_source_main_dir}")
                append_scala_files(scala_source_main_dir, md_file, base_project_dir)
                sources_processed_count +=1
            else:
                print(f"Warning: Scala source directory '{scala_source_main_dir}' not found.")

            # Process Scala files from the test source directory
            if os.path.isdir(scala_source_test_dir):
                print(f"Processing Scala files from: {scala_source_test_dir}")
                append_scala_files(scala_source_test_dir, md_file, base_project_dir)
                sources_processed_count +=1
            else:
                print(f"Warning: Scala source directory '{scala_source_test_dir}' not found.")

            # Append content from the specified Markdown file
            if os.path.isfile(markdown_to_include_path):
                print(f"Appending content from: {markdown_to_include_path}")
                append_markdown_content(markdown_to_include_path, md_file, base_project_dir)
                sources_processed_count +=1
            else:
                print(f"Warning: Markdown file to include '{markdown_to_include_path}' not found.")
        
        if sources_processed_count > 0:
            print(f"Successfully created/updated '{output_md_file_path}'")
        else:
            print(f"No new content found to add. '{output_md_file_path}' may be empty or unchanged if it existed.")

    except IOError as e:
        print(f"Error: Could not write to output file '{output_md_file_path}'. {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")