import abc
from core.util.interfaces import InterfaceMetaclass

class AfmInterface(metaclass=InterfaceMetaclass):
    """ Define the controls for an AFM."""
    _modtype = 'interface'
    _modclass = 'AfmInterface'

    @abc.abstractmethod
    def get_elevation(self):
        pass

    @abc.abstractmethod
    def close(self):
        pass
