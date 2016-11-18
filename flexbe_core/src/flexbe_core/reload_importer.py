# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import __builtin__
import os
import sys


class ReloadImporter:

    """Overrides the builtin import and automatically reloads all modules which are imported from on
    of the reload paths after calling enable."""

    def __init__(self):
        self._excluded_modules = sys.modules.keys()
        self._reload_paths = None
        self._import_stack = []
        self._reloaded_modules = set()
        self._import = __builtin__.__import__

    def enable(self):
        __builtin__.__import__ = self._reimport

    def disable(self):
        __builtin__.__import__ = self._import

    def add_reload_path(self, path):
        if self._reload_paths is None:
            self._reload_paths = tuple()
        self._reload_paths += (os.path.abspath(path),)

    def _reload(self, module):
        if module.__name__ not in self._import_stack and module.__name__ in sys.modules:
            if not self._import_stack:
                self._reloaded_modules.clear()
            self._import_stack.append(module.__name__)
            # force reload
            if module.__name__ not in self._reloaded_modules:
                reload(module)
                self._reloaded_modules.add(module.__name__)
            self._import_stack.pop()

    def _reimport(self, name, globals_=None, locals_=None, fromlist=None, level=-1):
        module = self._import(name, globals_, locals_, fromlist if not None else [], level if not None else -1)

        if module.__name__ not in self._excluded_modules and \
            (self._reload_paths is None or \
              (hasattr(module, '__file__') and len([p for p in self._reload_paths if module.__file__.startswith(p)]) > 0) \
            ):
            self._reload(module)

        return module
