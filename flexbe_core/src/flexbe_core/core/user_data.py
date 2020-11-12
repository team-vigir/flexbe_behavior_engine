#!/user/bin/env python
from copy import deepcopy
from flexbe_core.core.exceptions import UserDataError


class UserData(object):

    def __init__(self, reference=None, input_keys=None, output_keys=None, remap=None):
        self._data = dict()
        self._reference = reference if reference is not None else dict()
        self._input_keys = input_keys
        self._output_keys = output_keys
        self._remap = remap or dict()
        self._hashes = dict()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        for key, value in self._hashes.items():
            if value != hash(repr(self._data[key])):
                raise UserDataError("Illegally modified input-only key '%s', declare it as output." % key)

    def __contains__(self, key):
        if key in self._data:
            return True
        elif self._input_keys is not None and key not in self._input_keys:
            return False
        else:
            return self._remap.get(key, key) in self._reference

    def __getitem__(self, key):
        if key in self._data:
            return self._data[key]
        if key not in self:
            raise UserDataError("Key '%s' cannot be accessed, declare it as input key for read access." % key
                                if self._input_keys is not None and key not in self._input_keys else
                                "No data found for key '%s'" % key)
        value = self._reference[self._remap.get(key, key)]
        if self._output_keys is not None and key not in self._output_keys:
            self._data[key] = value
            self._hashes[key] = hash(repr(value))
            if getattr(value.__class__, "_has_header", False):
                # This is specific to rospy: If the value here is a message and has a header,
                #   it will automatically be modified during publishing by rospy.
                #   So to avoid hash issues, we need to return a copy.
                value = deepcopy(value)
        return value

    def __setitem__(self, key, value):
        if self._output_keys is not None and key in self._output_keys:
            self._reference[self._remap.get(key, key)] = value
        self._data[key] = value

    def __getattr__(self, key):
        if key.startswith('_'):
            return object.__getattr__(self, key)
        return self[key]

    def __setattr__(self, key, value):
        if key.startswith('_'):
            return object.__setattr__(self, key, value)
        if self._output_keys is not None and key not in self._output_keys:
            raise UserDataError("Key '%s' cannot be set, declare it as output key for write access." % key)
        self[key] = value

    def __call__(self, reference=None, add_from=None, update_from=None, remove_key=None):
        self._reference = reference or self._reference
        if isinstance(add_from, UserData):
            for key, value in add_from._data.items():
                if key not in self._data:
                    self._data[key] = value
        if isinstance(update_from, UserData):
            for key, value in update_from._data.items():
                self._data[key] = value
        if remove_key is not None and remove_key in self._data:
            del self._data[remove_key]

    def __len__(self):
        return len(self._data) + len(self._reference)

    def __str__(self):
        if isinstance(self._reference, UserData):
            data_str = '\n  '.join(str(self._reference).split('\n'))
        else:
            data_str = str(self._reference)
        return ("UserData object with %d data entries:\n"
                "  Input Keys: %s\n  Output Keys: %s\n  Data: %s\n  Remapping: %s\n  Reference: %s"
                % (len(self), str(self._input_keys), str(self._output_keys), str(self._data),
                   str(self._remap), data_str))
