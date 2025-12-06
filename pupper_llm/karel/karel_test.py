# karel_test.py

import karel


def main():
    pupper = karel.KarelPupper()
    pupper.move_forward()
    pupper.wiggle()
    pupper.move_backward()
    pupper.wiggle()
    pupper.turn_right()
    pupper.turn_left()
    
    # pupper.wiggle()
    # pupper.bob()
    # pupper.dance()
    # pupper.bark()
    # pupper.stop()
    

if __name__ == '__main__':
    main()
