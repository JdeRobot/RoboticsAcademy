import re
import subprocess


class Lint:

    def evaluate_code(self, code, warnings=False):
        try:
            code = re.sub(r'from HAL import HAL', 'from hal import HAL', code)
            code = re.sub(r'from GUI import GUI', 'from gui import GUI', code)
            code = re.sub(r'from MAP import MAP', 'from map import MAP', code)
            code = re.sub(r'\nimport cv2\n', '\nfrom cv2 import cv2\n', code)

            # Avoids EOF error when iterative code is empty (which prevents other errors from showing)
            while_position = re.search(
                r'[^ ]while\s*\(\s*True\s*\)\s*:|[^ ]while\s*True\s*:|[^ ]while\s*1\s*:|[^ ]while\s*\(\s*1\s*\)\s*:', code)
            sequential_code = code[:while_position.start()]
            iterative_code = code[while_position.start():]
            iterative_code = re.sub(
                r'[^ ]while\s*\(\s*True\s*\)\s*:|[^ ]while\s*True\s*:|[^ ]while\s*1\s*:|[^ ]while\s*\(\s*1\s*\)\s*:', '\n', iterative_code, 1)
            iterative_code = re.sub(r'^[ ]{4}', '', iterative_code, flags=re.M)
            code = sequential_code + iterative_code

            f = open("user_code.py", "w")
            f.write(code)
            f.close()

            open("user_code.py", "r")

            command = "export PYTHONPATH=$PYTHONPATH:/$EXERCISE_FOLDER/web-template; python3 RoboticsAcademy/src/manager/lint/pylint_checker.py"
            ret = subprocess.run(command, capture_output=True, shell=True)
            result = ret.stdout.decode()
            result = result + "\n"

            # Removes convention, refactor and warning messages
            if not warnings:
                convention_messages = re.search(
                    ":[0-9]+: convention.*\n", result)
                while (convention_messages != None):
                    result = result[:convention_messages.start(
                    )] + result[convention_messages.end():]
                    convention_messages = re.search(
                        ":[0-9]+: convention.*\n", result)
                warning_messages = re.search(":[0-9]+: warning.*\n", result)
                while (warning_messages != None):
                    result = result[:warning_messages.start()] + \
                        result[warning_messages.end():]
                    warning_messages = re.search(
                        ":[0-9]+: warning.*\n", result)
                refactor_messages = re.search(":[0-9]+: refactor.*\n", result)
                while (refactor_messages != None):
                    result = result[:refactor_messages.start()] + \
                        result[refactor_messages.end():]
                    refactor_messages = re.search(
                        ":[0-9]+: refactor.*\n", result)

            # Removes unexpected EOF error
            eof_exception = re.search(":[0-9]+: error.*EOF.*\n", result)
            if (eof_exception != None):
                result = result[:eof_exception.start()] + \
                    result[eof_exception.end():]

            # Removes no value for argument 'self' error
            self_exception = re.search(
                ":[0-9]+:.*value.*argument.*unbound.*method.*\n", result)
            while (self_exception != None):
                result = result[:self_exception.start()] + \
                    result[self_exception.end():]
                self_exception = re.search(
                    ":[0-9]+:.*value.*argument.*unbound.*method.*\n", result)

            # Removes assignment from no return error
            self_exception = re.search(":[0-9]+:.*E1111.*\n", result)
            while (self_exception != None):
                result = result[:self_exception.start()] + \
                    result[self_exception.end():]
                self_exception = re.search(":[0-9]+:.*E1111.*\n", result)

            # Removes E1136 until issue https://github.com/PyCQA/pylint/issues/1498 is closed
            self_exception = re.search(":[0-9]+:.*E1136.*\n", result)
            while (self_exception != None):
                result = result[:self_exception.start()] + \
                    result[self_exception.end():]
                self_exception = re.search(":[0-9]+:.*E1136.*\n", result)

            # Returns an empty string if there are no errors
            error = re.search("error", result)
            if (error == None and not warnings):
                return ""
            else:
                return result.strip()
        except Exception as ex:
            print(ex)
