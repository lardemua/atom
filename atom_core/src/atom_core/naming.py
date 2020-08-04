def generateName(name, prefix='', suffix='', separator='_'):
    """ Standardized form of deriving a name with a prefix or a suffix with <separator> separating them. """

    if prefix:
        prefix = prefix + separator

    if suffix:
        suffix = separator + suffix
    return str(prefix) + str(name) + str(suffix)


def generateKey(parent, child, suffix=''):
    return parent + '-' + child + suffix