import json, os # , sys

def convert_ipynb_to_py(ipynb_file, output_file=None):
    if not ipynb_file.endswith('.ipynb'):
        print("Please provide a valid .ipynb file.")
        return

    try:
        with open(ipynb_file, 'r', encoding='utf-8') as f:
            notebook = json.load(f)
        
        code_lines = []
        for cell in notebook.get('cells', []):
            if cell.get('cell_type') == 'code':
                # code_lines.append("# Cell\n")
                code_lines.extend(cell.get('source', []))
                code_lines.append("\n\n")
        
        if not code_lines:
            print("No code cells found in the notebook.")
            return
        
        # Determine the output file name
        if output_file is None:
            output_file = os.path.splitext(ipynb_file)[0] + ".py"
        
        with open(output_file, 'w', encoding='utf-8') as f:
            f.writelines(code_lines)
        
        print(f"Conversion complete. Python file saved as: {output_file}")
    except Exception as e:
        print(f"Error converting file: {e}")

# Example usage:
convert_ipynb_to_py(
    ipynb_file = r'C:\Users\JonathanChackoPattas\OneDrive - Maritime Support Solutions\Desktop\Class Notes\Seneca\AIG150 - Data Preparation and Governance\Week 5\Lab\AIG150_Lab5-JonathanChacko.ipynb', 
    output_file = r"C:\Users\JonathanChackoPattas\OneDrive - Maritime Support Solutions\Desktop\Class Notes\Seneca\Converted Files\Lab5.py"
)

# if __name__ == "__main__":
#     if len(sys.argv) < 2:
#         print("Usage: python convert_ipynb_to_py.py <notebook_file.ipynb> [output_file.py]")
#     else:
#         ipynb_file = sys.argv[1]
#         output_file = sys.argv[2] if len(sys.argv) > 2 else None
#         convert_ipynb_to_py(ipynb_file, output_file)
