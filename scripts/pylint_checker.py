import tempfile
import re
from pylint import epylint as lint

code = open('user_code.py')
python_code = code.read()
code.close()

code_file = tempfile.NamedTemporaryFile(delete=False)
code_file.write(python_code.encode())
code_file.seek(0)
options = code_file.name + ' --enable=similarities' + " --disable=C0114,C0116"
(stdout, stderr) = lint.py_run(options, return_std=True)
code_file.seek(0)
code_file.close()
result = stdout.getvalue()
name = code_file.name.split('/')[-1]
result = result[(result.find(name) + len(name) - 1):result.find('---')]
result = result.replace(code_file.name, '')
result = result[result.find('\n'):]

print(result)
