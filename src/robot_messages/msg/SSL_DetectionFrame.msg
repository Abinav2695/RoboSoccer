uint32             frame_number 
float64            t_capture     
float64            t_sent        
uint32             camera_id     
SSL_DetectionBall[]  balls         
SSL_DetectionRobot[] robots_yellow 
SSL_DetectionRobot[] robots_blue   


#example of ssl detection frame obtained from ssl_client
# {
#   "detection": {
#     "frame_number": 377308,
#     "t_capture": 13977.820277,
#     "t_sent": 1505683703.172275,
#     "camera_id": 1,
#     "balls": [
#       {
#         "confidence": 0.930344820022583,
#         "area": 78,
#         "x": -14.979391098022461,
#         "y": -12.432811737060547,
#         "z": 0,
#         "pixel_x": 276.5769348144531,
#         "pixel_y": 225.73077392578125
#       }
#       "{... other balls ...}"
#     ],
#     "robots_yellow": [
#       {
#         "confidence": 0.930344820022583,
#         "area": 78,
#         "x": -14.979391098022461,
#         "y": -12.432811737060547,
#         "z": 0,
#         "pixel_x": 276.5769348144531,
#         "pixel_y": 225.73077392578125
#       }
#       "{... other yellow robots ...}"
#     ],
#     "robots_blue": [
#       {
#         "confidence": 0.930344820022583,
#         "area": 78,
#         "x": -14.979391098022461,
#         "y": -12.432811737060547,
#         "z": 0,
#         "pixel_x": 276.5769348144531,
#         "pixel_y": 225.73077392578125
#       }
#       "{... other blue robots ...}"
#     ]
#   }
# }
