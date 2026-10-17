"""Same key precedence and normalized wheel mixing as the real dashboard.

Physical wheel-forward duty is the negative of the hardware PWM polarity.
"""
import itertools


def vector(keys):
    k=set(keys.upper())
    f=int('W' in k)-int('S' in k)
    pivot=int('Q' in k)-int('E' in k)
    steering=int('A' in k)-int('D' in k)
    return f, pivot or steering


def wheel_duties(keys, pwm):
    f,t=vector(keys)
    left,right=-f+t,-f-t
    scale=max(1,abs(left),abs(right))
    return dict(zip(('front-left','rear-left','front-right','rear-right'),
                    [round(left/scale*pwm)]*2+[round(right/scale*pwm)]*2))


def combinations():
    return {''.join(k): dict(zip(('forward','turn'),vector(''.join(k))))
            for count in range(7) for k in itertools.combinations('WASDQE',count)}
