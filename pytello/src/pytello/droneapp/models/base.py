class Singleton(type):
    """
    Una metaclase que permite que una clase tenga solo una instancia.

    Asegura que solo se cree una instancia de una clase y
    proporciona un punto de acceso global a esa instancia.

    Uso:
    class MiClase(metaclass=Singleton):
        # definición de la clase

    """
    
    _instancias = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instancias:
            cls._instancias[cls] = super(
                Singleton, cls).__call__(*args, **kwargs)
        return cls._instancias[cls]
