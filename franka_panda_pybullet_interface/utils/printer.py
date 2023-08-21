import os
import time
from datetime import datetime

from termcolor import cprint

from npm_base.utils import load_txt, write_txt


class Printer:
    def __init__(self, source, experiment_name='', save=False):
        self.source = source  # where is the logger object instantiated? could be a class, function, custom tag, etc.
        self.save_log = save
        if self.save_log:
            assert experiment_name != ''

            self.log_dir = '%s/npm/logs/print_logs' % os.path.expanduser('~')
            self.log_file = '%s/%s-%s.txt' % \
                            (self.log_dir, experiment_name, datetime.now().strftime("%Y%m%d_%H%M%S"))
            self.log_str = ''

    def print_info(self, msg, color='', suppress_info=False):
        if suppress_info:
            log_str = f'[INFO] {msg}'
        else:
            log_str = f'[{time.time()}] [INFO] [{self.source}] {msg}'
        if self.save_log:
            self.log_str += log_str + '\n'

        if len(color) > 0:
            cprint(log_str, color)
        else:
            print(log_str)

    def print_success(self, msg, suppress_info=False):
        if suppress_info:
            log_str = f'[SUCCESS] {msg}'
        else:
            log_str = f'[{time.time()}] [SUCCESS] [{self.source}] {msg}'
        if self.save_log:
            self.log_str += log_str + '\n'
        cprint(log_str, 'green', attrs=['bold'])

    def print_warning(self, msg, suppress_info=False):
        if suppress_info:
            log_str = f'[WARNING] {msg}'
        else:
            log_str = f'[{time.time()}] [WARNING] [{self.source}] {msg}'
        if self.save_log:
            self.log_str += log_str + '\n'
        cprint(log_str, 'yellow')

    def print_error(self, msg, suppress_info=False):
        if suppress_info:
            log_str = f'[ERROR] {msg}'
        else:
            log_str = f'[{time.time()}] [ERROR] [{self.source}] {msg}'
        if self.save_log:
            self.log_str += log_str + '\n'
        cprint(log_str, 'red', attrs=['bold'])

    def save(self):
        assert self.save_log, 'Logger is not set to save.'

        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)

        write_txt(self.log_file, self.log_str)
        self.print_info('Print log data saved to %s' % self.log_file, color='blue')

    def load(self):
        assert os.path.isfile(self.log_file), 'Log file does not exist.'

        return load_txt(self.log_file)
