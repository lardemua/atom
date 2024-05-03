
# Standard imports
import os
import pty
import re
import subprocess
import sys

from colorama import Fore, Style, Back
from pytictoc import TicToc



def execute(cmd, blocking=True, verbose=True, save_path=None, save_filename_additions=''):
    """ @brief Executes the command in the shell in a blocking or non-blocking manner
        @param cmd a string with teh command to execute
        @return
    """

    # Open files to save stdout and stderr if save_path is provided
    stdout_file = None
    stderr_file = None

    if save_path:
        stdout_filename = save_path + '/' + save_filename_additions + 'stdout.txt'
        stderr_filename = save_path + '/' + save_filename_additions + 'stderr.txt'
        stdout_file = open(stdout_filename, 'w')
        stderr_file = open(stderr_filename, 'w')

    # Print the command being executed and measure the execution time if verbose is True
    if verbose:
        print("Executing command: " + cmd)
        tictoc = TicToc()
        tictoc.tic()

    # Execute the command in the shell using subprocess.Popen
    proc = subprocess.Popen(cmd, shell=True, universal_newlines=True, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE)
    
    stdout_data = ''
    # Read the stdout of the process and print it in real-time
    if blocking:
        while True:
            output = proc.stdout.read(1)
            if output == '' and proc.poll() is not None:
                break
            if output:
                if verbose:
                    print(output, end='')
                sys.stdout.flush()
                stdout_data += output

        # Write the stdout data to the file if stdout_file is provided
        if stdout_file:
            stdout_file.write(removeColorsFromText(stdout_data))
            stdout_file.close()

    # Wait for the command to finish and get the stderr data
    stdout_data, stderr_data = proc.communicate()

    if not blocking:
        # Write the stdout data to the file if stdout_file is provided
        if stdout_file:
            stdout_file.write(removeColorsFromText(stdout_data))
            stdout_file.close()

    # If the return code is not 0, print the stderr data and exit
    if not proc.returncode == 0:
        print(Fore.RED + Back.YELLOW + 'Error running command. stderr is:' + Style.RESET_ALL)
        print(stderr_data)
        if stderr_file:
            stderr_file.write(removeColorsFromText(stderr_data))
            stderr_file.close()
        if stdout_file:
            stdout_file.close()
        exit(0)
    
    # Print the execution time if verbose is True
    if verbose:
        toc = str(round(tictoc.tocvalue(), 5))
        print(Fore.BLUE + 'Command executed in ' + toc + ' secs.' + Style.RESET_ALL)
    
    # Close the stdout and stderr files if provided
    if stdout_file:
        stdout_file.close()
    if stderr_file:
        stderr_file.close()
    

def removeColorsFromText(text):
    # Must remove ansi escape characters so that its possible to convert to float
    # https://stackoverflow.com/questions/14693701/how-can-i-remove-the-ansi-escape-sequences-from-a-string-in-python
    ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
    text_without_color_codes = ansi_escape.sub('', text)
    return text_without_color_codes


def execColored(cmd):
    """Using popen it is not possible to maintain colors as those used in colorama.
    This way of making system calls preserves colors.
    https://stackoverflow.com/questions/56835373/python-subprocess-realtime-print-with-colors-and-save-stdout
    Args:
        cmd (str): The command to execute
    """

    cmd_parts = cmd.split(" ")
    pty.spawn(cmd_parts, lambda fd: os.read(fd, 1024))


def resolvePath(path, verbose=False):
    """ Resolves path by replacing environment variables, common notations (e.g. ~ for home/user)"""

    path = os.path.expanduser(path)
    path = os.path.expandvars(path)
    path = os.path.abspath(path)
    path = os.path.normpath(path)
    return path


def expandToLaunchEnv(path):
    if len(path) == 0:  # if path is empty, path[0] does not exist
        return path

    if path[0] == '~':
        path = '$(env HOME)' + path[1:]

    if '$' not in path:
        return path

    evars = re.compile(r'\$(\w+|\{[^}]*\})')
    i = 0
    while True:
        m = evars.search(path, i)
        if not m:
            break

        i, j = m.span(0)
        name = m.group(1)
        if name.startswith('{') and name.endswith('}'):
            name = name[1:-1]

        tail = path[j:]
        path = path[:i] + '$(env {})'.format(name)
        i = len(path)
        path += tail

    return path
