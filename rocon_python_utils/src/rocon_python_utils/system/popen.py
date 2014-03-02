#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_tools/license/LICENSE
#

##############################################################################
# Imports
##############################################################################

# system
import os
import threading
import subprocess
import signal

##############################################################################
# Subprocess
##############################################################################


class Popen(object):
    '''
      Use this if you want to attach a postexec function to popen (which
      is not supported by popen at all). It also defaults setting and
      terminating whole process groups (something we do quite often in ros).
    '''
    __slots__ = [
            '_proc',
            '_thread',
            '_external_preexec_fn',
            '_shell',
            'terminate'
        ]

    def __init__(self, popen_args, shell=False, preexec_fn=None, postexec_fn=None):
        '''
          @param popen_args : list/tuple of usual popen args
          @type list/tuple

          @param shell : same as the shell argument passed to subprocess.Popen
          @type bool

          @param preexec_fn : usual popen pre-exec function
          @type method with no args

          @param postexec_fn : the callback which we support for postexec.
          @type method with no args
        '''
        self._proc = None
        self._shell = shell
        self._external_preexec_fn = preexec_fn
        self._thread = threading.Thread(target=self._run_in_thread, args=(popen_args, self._preexec_fn, postexec_fn))
        self._thread.start()

    def send_signal(self, sig):
        os.killpg(self._proc.pid, sig)
        # This would be the normal way if not defaulting settings for process groups
        #self._proc.send_signal(sig)

    def _preexec_fn(self):
        os.setpgrp()
        if self._external_preexec_fn is not None:
            self._external_preexec_fn()

    def terminate(self):
        '''
          @raise OSError if the process has already shut down.
        '''
        return os.killpg(self._proc.pid, signal.SIGTERM)
        # if we were not setting process groups
        #return self._proc.terminate() if self._proc is not None else None

    def _run_in_thread(self, popen_args, preexec_fn, postexec_fn):
        '''
          Worker function for the thread, creates the subprocess itself.
        '''
        if preexec_fn is not None:
            if self._shell == True:
                #print("rocon_python_utils.os.Popen: %s" % " ".join(popen_args))
                self._proc = subprocess.Popen(" ".join(popen_args), shell=True, preexec_fn=preexec_fn)
            else:
                #print("rocon_python_utils.os..Popen: %s" % popen_args)
                self._proc = subprocess.Popen(popen_args, shell=self._shell, preexec_fn=preexec_fn)
        else:
            self._proc = subprocess.Popen(popen_args, shell=self._shell)
        self._proc.wait()
        if postexec_fn is not None:
            postexec_fn()
        return
