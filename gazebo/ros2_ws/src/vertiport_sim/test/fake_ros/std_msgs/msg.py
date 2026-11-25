class String:
    __slots__ = ('data',)

    def __init__(self, data=''):
        if not isinstance(data, str):
            raise TypeError('String.data must be str')
        self.data = data
