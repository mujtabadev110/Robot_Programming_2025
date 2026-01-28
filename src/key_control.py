
class Target:
    def __init__(self, x, y, z, step=0.02):
        self.x = x
        self.y = y
        self.z = z
        self.step = step


# keypress handler
def make_key_callback(target: Target):
    def key_callback(keycode):
        
        if keycode == 265 : target.z += target.step  # Up
        elif keycode == 264 : target.z -= target.step  # Down
        elif keycode == 263 : target.x += target.step  # Right
        elif keycode == 262 : target.x -= target.step  # Left
        
        else:
            try:
                key = chr(keycode).upper()
                if key == 'B':
                    target.y += target.step  # Positive Y
                elif key == 'V':
                    target.y -= target.step  # Negative Y
            except (ValueError, OverflowError):
                pass
    return key_callback
