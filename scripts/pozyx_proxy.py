from pypozyx import POZYX_SUCCESS, POZYX_FAILURE, POZYX_TIMEOUT, SingleRegister
import pypozyx
from threading import RLock


class PozyxException(Exception):
    pass


class PozyxExceptionUnknown(Exception):
    def __init__(self, fun):
        self.message = 'Unknown error in %s' % fun


class PozyxExceptionTimeout(PozyxException):
    def __init__(self, fun):
        self.message = 'Timeout in %s' % fun


class PozyxExceptionNoConnection(PozyxException):
    def __init__(self, fun):
        self.message = 'No connection in %s' % fun


error_codes = {
    'POZYX_ERROR_NONE':
        "",
    'POZYX_ERROR_I2C_WRITE':
        "Error 0x01: Error writing to a register through the I2C bus",
    'POZYX_ERROR_I2C_CMDFULL':
        "Error 0x02: Pozyx cannot handle all the I2C commands at once",
    'POZYX_ERROR_ANCHOR_ADD':
        "Error 0x03: Cannot add anchor to the internal device list",
    'POZYX_ERROR_COMM_QUEUE_FULL':
        "Error 0x04: Communication queue is full, too many UWB messages",
    'POZYX_ERROR_I2C_READ':
        "Error 0x05: Error reading from a register from the I2C bus",
    'POZYX_ERROR_UWB_CONFIG':
        "Error 0x06: Cannot change the UWB configuration",
    'POZYX_ERROR_OPERATION_QUEUE_FULL':
        "Error 0x07: Pozyx cannot handle all the operations at once",
    'POZYX_ERROR_STARTUP_BUSFAULT':
        "Error 0x08: Internal bus error",
    'POZYX_ERROR_FLASH_INVALID':
        "Error 0x09: Flash memory is corrupted or invalid",
    'POZYX_ERROR_NOT_ENOUGH_ANCHORS':
        "Error 0x0A: Not enough anchors available for positioning",
    'POZYX_ERROR_DISCOVERY':
        "Error 0x0B: Error during the Discovery process",
    'POZYX_ERROR_CALIBRATION':
        "Error 0x0C: Error during the auto calibration process",
    'POZYX_ERROR_FUNC_PARAM':
        "Error 0x0D: Invalid function parameters for the register function",
    'POZYX_ERROR_ANCHOR_NOT_FOUND':
        "Error 0x0E: The coordinates of an anchor are not found",
    'POZYX_ERROR_GENERAL':
        "Error 0xFF: General error"}

pozyx_exceptions = {}

for e, m in error_codes.items():
    value = getattr(pypozyx, e)
    suffix = ''.join(map(str.title, e.split('_')[2:]))
    name = "PozyxException%s" % suffix

    class ex(PozyxException):
        def __init__(self, function, m=m):
            self.message = '%s in %s' % (m, function)
        pass
    ex.__name__ = name
    pozyx_exceptions[value] = ex
    vars()[name] = ex


class PozyxProxy(object):
    def __init__(self, pozyx, remote_id=None):
        self.pozyx = pozyx
        self.remote_id = remote_id
        self.lock = RLock()

    def raise_error(self, fun):
        pozyx = self.pozyx
        remote_id = self.remote_id
        error_code = SingleRegister()
        if remote_id is None:
            pozyx.getErrorCode(error_code)
            self.lock.release()
            c = error_code.data[0]
            raise pozyx_exceptions.get(c, PozyxExceptionUnknown)(fun)
        status = pozyx.getErrorCode(error_code, remote_id)
        if status == POZYX_SUCCESS:
            self.lock.release()
            c = error_code.data[0]
            raise pozyx_exceptions.get(c, PozyxExceptionUnknown)(fun)
        else:
            pozyx.getErrorCode(error_code)
            self.lock.release()
            raise PozyxExceptionNoConnection(fun)

    def __getattr__(self, attr):
        a = getattr(self.pozyx, attr)
        if callable(a):
            def f(*args, **kwargs):
                if self.lock.acquire(False):
                    try:
                        status = a(*args, **kwargs)
                    except ValueError:
                        self.lock.release()
                        raise PozyxExceptionUnknown(attr)
                    except Exception:
                        self.lock.release()
                        raise PozyxExceptionUnknown(attr)
                    if status == POZYX_FAILURE:
                        self.raise_error(attr)
                    self.lock.release()
                    if status == POZYX_TIMEOUT:
                        raise PozyxExceptionTimeout(attr)
                    return True
                return False
            return f
        return a
