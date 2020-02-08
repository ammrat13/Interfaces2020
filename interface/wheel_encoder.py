import Jetson.GPIO as GPIO

def pulsein(channel, value, timeout=1000):
    """
    :param channel: the pin to check pulse on
    :param value:   the value of the pin to begin measuring with
    :param timeout: timeout in milliseconds to give up after
    """
    first_edge, second_edge = None
    if value == GPIO.HIGH:
        first_edge, second_edge = GPIO.RISING, GPIO.FALLING
    else:
        first_edge, second_edge = GPIO.FALLING, GPIO.RISING
    GPIO.wait_for_edge(channel, first_edge)
    start = time.time_ns()
    GPIO.wait_for_edge(channel, second_edge)
    end = time.time_ns()
    return (end - start) // 1000