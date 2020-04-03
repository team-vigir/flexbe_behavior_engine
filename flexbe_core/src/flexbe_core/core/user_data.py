#!/user/bin/env python
from flexbe_core.core.exceptions import UserDataError


class UserData(object):

    def __init__(self, reference=None, input_keys=None, output_keys=None, remap=None):
        self._data = reference._data if reference is not None else dict()
        self._input_keys = input_keys
        self._output_keys = output_keys
        self._remap = remap or dict()

    def __contains__(self, key):
        if self._input_keys is not None and key not in self._input_keys:
            return False
        else:
            return self._remap.get(key, key) in self._data

    def __getitem__(self, key):
        if key not in self:
            raise UserDataError("Key '%s' cannot be accessed, declare it as input key for read access." % key
                                if self._input_keys is not None and key not in self._input_keys else
                                "No data found for declared input key '%s'" % key)
        return self._data[self._remap.get(key, key)]

    def __setitem__(self, key, value):
        if self._output_keys is not None and key not in self._output_keys:
            raise UserDataError("Key '%s' cannot be set, declare it as output key for write access." % key)
        self._data[self._remap.get(key, key)] = value

    def __getattr__(self, key):
        if key.startswith('_'):
            return object.__getattr__(self, key)
        return self[key]

    def __setattr__(self, key, value):
        if key.startswith('_'):
            return object.__setattr__(self, key, value)
        self[key] = value
