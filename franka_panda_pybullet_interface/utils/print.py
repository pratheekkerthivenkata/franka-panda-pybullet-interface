import os
import time
from datetime import datetime

from termcolor import cprint


class Print:
    def __init__(self, source, experiment_name=''):
        self.source = source  # where is the logger object instantiated? could be a class, function, custom tag, etc.

    def print_info(self, msg, color=''):
        log_str = f'[{time.time()}] [INFO] [{self.source}] {msg}'
        if len(color) > 0:
            cprint(log_str, color)
        else:
            print(log_str)

    def print_success(self, msg):
        log_str = f'[{time.time()}] [SUCCESS] [{self.source}] {msg}'
        cprint(log_str, 'green', attrs=['bold'])

    def print_warning(self, msg):
        log_str = f'[{time.time()}] [WARNING] [{self.source}] {msg}'
        cprint(log_str, 'yellow')

    def print_error(self, msg):
        log_str = f'[{time.time()}] [ERROR] [{self.source}] {msg}'
        cprint(log_str, 'red', attrs=['bold'])
