import os

def create_markdown_from_scala_files(source_directory, output_markdown_file):
    """
    Finds all Scala files in the source_directory and its subdirectories,
    and concatenates their content into a single Markdown file.

    Args:
        source_directory (str): The path to the directory containing Scala files.
        output_markdown_file (str): The name of the Markdown file to create.
    """
    with open(output_markdown_file, 'w', encoding='utf-8') as md_file:
        for root, _, files in os.walk(source_directory):
            for file_name in files:
                if file_name.endswith(".scala"):
                    file_path = os.path.join(root, file_name)
                    relative_file_path = os.path.relpath(file_path, source_directory)
                    
                    md_file.write(f"## File: {relative_file_path}\n\n")
                    md_file.write("```scala\n")
                    
                    try:
                        with open(file_path, 'r', encoding='utf-8') as scala_file:
                            md_file.write(scala_file.read())
                    except Exception as e:
                        md_file.write(f"// Error reading file: {e}\n")
                    
                    md_file.write("\n```\n\n")
    print(f"Successfully created '{output_markdown_file}' with Scala files from '{source_directory}'")

if __name__ == "__main__":
    # Define the source directory containing your Scala files
    # Please adjust this path if your script is not in the parent directory of 'generators'
    source_dir = "/home/lzy/chipyard/generators/mycnnaccelerators/src/main/scala/mycnnaccelerators"
    
    # Define the name for your output Markdown file
    output_md = "/home/lzy/chipyard/generators/mycnnaccelerators/src/main/scala/mycnnaccelerators/mycnnaccelerators_scala_files.md"

    if os.path.isdir(source_dir):
        create_markdown_from_scala_files(source_dir, output_md)
    else:
        print(f"Error: Source directory '{source_dir}' not found.")
        print("Please ensure the path is correct and the script is run from a location")
        print("where this relative path is valid, or use an absolute path.")