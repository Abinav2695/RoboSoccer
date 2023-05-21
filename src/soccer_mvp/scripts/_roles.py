#!/usr/bin/env python3

class Init:
    
    def __init__(self, d):

        self.d = d


class Roles(Init):

    def __init__(self, a, b, c):

        self.a = a
        self.b = b
        self.c = c



class Attacker(Roles):

    def __init__(self, a, b, c, d):

        super().__init__(Roles, d)
        super().__init__(a, b, c)

    
    def __str__(self):

        return str(self.a+self.b+self.c+self.d)


if __name__=='__main__':

    attacker = Attacker(10, 20, 30, 40)
    print(attacker)

