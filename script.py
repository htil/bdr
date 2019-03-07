from LineError import *
import time

if __name__ == '__main__':
    le = LineError(1)

    sl = cv2.imread('sl.jpg')
    sr = cv2.imread('sr.jpg')
    sc = cv2.imread('straight_center.jpg')
    l = cv2.imread('l.jpg')
    r = cv2.imread('r.jpg')

    '''
    delta, theta = le.GetError(sl)
    print(delta, theta)

    delta, theta = le.GetError(sr)
    print(delta, theta)

    delta, theta = le.GetError(sc)
    print(delta, theta)
    '''

    delta, theta = le.GetError(sc)
    print(delta, theta)
    cv2.imwrite('result_c.jpg', le.GetImage())

    '''
    delta, theta = le.GetError(r)
    print(delta, theta)
    cv2.imwrite('result_r.jpg', le.GetImage())
    '''
