"""Service node to manage zoom for cameras."""
import time

import rclpy
from rclpy.node import Node

from reachy_msgs.srv import GetCameraZoomLevel, SetCameraZoomLevel
from reachy_msgs.srv import GetCameraZoomSpeed, SetCameraZoomSpeed
from reachy_msgs.srv import GetCameraZoomFocus, SetCameraZoomFocus
from reachy_msgs.srv import SetFocusState

from zoom_kurokesu import ZoomController


class ZoomControllerService(Node):
    """Main node creating the zoom services for cameras."""

    def __init__(self, default_zoom_speed: int = 10000, default_zoom_level: str = 'inter') -> None:
        """Set up the node and create the services."""
        super().__init__('camera_zoom_controller_service')
        self.logger = self.get_logger()

        self.controller = ZoomController('/dev/kurokesu')
        self.controller.set_speed(default_zoom_speed)
        # for side in ('left', 'right'):
        #     self.controller.homing(side)
        #     self.controller.set_zoom_level(side, default_zoom_level)

        self.current_zoom_info = {
            'left_eye': {
                'zoom': self.controller.zoom_pos["left"][default_zoom_level]['zoom'],
                'focus': self.controller.zoom_pos["left"][default_zoom_level]['focus'],
                'speed': default_zoom_speed,
                'zoom_level': default_zoom_level,
            },
            'right_eye': {
                'zoom': self.controller.zoom_pos["left"][default_zoom_level]['zoom'],
                'focus': self.controller.zoom_pos["left"][default_zoom_level]['focus'],
                'speed': default_zoom_speed,
                'zoom_level': default_zoom_level,
            },
        }

        self.get_zoom_level_service = self.create_service(
            GetCameraZoomLevel,
            'get_camera_zoom_level',
            self.get_zoom_level_callback,
        )
        self.logger.info(f'Launching "{self.get_zoom_level_service.srv_name}" service.')

        self.set_command_service = self.create_service(
            SetCameraZoomLevel,
            'set_camera_zoom_level',
            self.set_zoom_command_callback,
        )
        self.logger.info(f'Launching "{self.set_command_service.srv_name}" service.')

        self.get_speed_service = self.create_service(
            GetCameraZoomSpeed,
            'get_camera_zoom_speed',
            self.get_zoom_speed_callback,
        )

        self.set_speed_service = self.create_service(
            SetCameraZoomSpeed,
            'set_camera_zoom_speed',
            self.set_zoom_speed_callback,
        )
        self.logger.info(f'Launching "{self.set_speed_service.srv_name}" service.')

        self.get_multiple_zoom_focus_service = self.create_service(
            GetCameraZoomFocus,
            'get_camera_zoom_focus',
            self.get_camera_zoom_focus_callback,
        )

        self.set_multiple_zoom_focus_service = self.create_service(
            SetCameraZoomFocus,
            'set_camera_zoom_focus',
            self.set_camera_zoom_focus_callback,
        )

        self.set_focus_state = self.create_client(
            SetFocusState,
            'set_focus_state',
        )

        self.logger.info('Node ready!')

    def get_zoom_level_callback(self,
                                request: GetCameraZoomLevel.Request,
                                response: GetCameraZoomLevel.Response,
                                ) -> GetCameraZoomLevel.Response:
        """Get the current camera zoom level."""
        if request.name not in ['left_eye', 'right_eye']:
            self.logger.warning("Invalid name sent to zoom controller (must be in ('left_eye', 'right_eye')).")
            return response
        response.zoom_level = self.current_zoom_info[request.name]['zoom_level']
        return response

    def set_zoom_command_callback(self,
                                  request: SetCameraZoomLevel.Request,
                                  response: SetCameraZoomLevel.Response,
                                  ) -> SetCameraZoomLevel.Response:
        """Handle set_camera_zoom_level request."""
        try:
            eye_side = {
                'left_eye': 'left',
                'right_eye': 'right',
            }[request.name]
        except KeyError:
            self.logger.warning("Invalid name sent to zoom controller (must be in ('left_eye', 'right_eye')).")
            response.success = False
            return response

        if request.zoom_level == 'homing':
            self.controller.homing(eye_side)
            self.current_zoom_info[request.name]['zoom_level'] = 'zero'

        elif request.zoom_level in ('in', 'out', 'inter'):
            self.controller.set_zoom_level(eye_side, request.zoom_level)
            self.current_zoom_info[request.name]['zoom_level'] = request.zoom_level
            self.current_zoom_info[request.name]['zoom'] = int(self.controller.zoom_pos[eye_side][request.zoom_level]['zoom'])

        else:
            self.logger.warning("Invalid command sent to zoom controller (must be in ('homing', 'in', 'out' or 'inter')).")
            response.success = False
            return response

        response.success = True
        return response

    def get_zoom_speed_callback(self,
                                request: GetCameraZoomSpeed.Request,
                                response: GetCameraZoomSpeed.Response,
                                ) -> GetCameraZoomSpeed.Response:
        """Get the current camera zoom speed."""
        if request.name not in ['left_eye', 'right_eye']:
            self.logger.warning("Invalid name sent to zoom controller (must be in ('left_eye', 'right_eye')).")
            return response

        response.speed = self.current_zoom_info[request.name]['speed']
        return response

    def set_zoom_speed_callback(self,
                                request: SetCameraZoomSpeed.Request,
                                response: SetCameraZoomSpeed.Response,
                                ) -> SetCameraZoomSpeed.Response:
        """Handle set_camera_zoom_speed request."""
        if request.name not in ['left_eye', 'right_eye']:
            self.logger.warning("Invalid name sent to zoom controller (must be in ('left_eye', 'right_eye')).")
            response.success = False
            return response

        self.controller.set_speed(request.speed)
        self.current_zoom_info[request.name] = request.speed

        response.success = True
        return response

    def get_camera_zoom_focus_callback(self,
                                       request: GetCameraZoomFocus.Request,
                                       response: GetCameraZoomFocus.Response,
                                       ) -> GetCameraZoomFocus.Response:
        """Handle get_camera_zoom_focus callback."""
        response.left_focus = self.current_zoom_info['left_eye']['focus']
        response.left_zoom = self.current_zoom_info['left_eye']['zoom']
        response.right_focus = self.current_zoom_info['right_eye']['focus']
        response.right_zoom = self.current_zoom_info['right_eye']['zoom']
        return response

    def set_camera_zoom_focus_callback(self,
                                       request: SetCameraZoomFocus.Request,
                                       response: SetCameraZoomFocus.Response,
                                       ) -> SetCameraZoomFocus.Response:
        """Handle set_camera_zoom_focus callback."""
        command = {'left': {}, 'right': {}}

        for cmd_name in list(request.get_fields_and_field_types().keys()):
            cmd = getattr(request, cmd_name)
            if cmd.flag:
                side, cmd_type = cmd_name.split('_')
                command[side][cmd_type] = cmd.value
                self.current_zoom_info[side+'_eye'][cmd_type] = cmd.value

        self.controller._send_custom_command(command)
        response.success = True
        return response

    def start_autofocus(self):
        """Call set_focus_state service."""
        req = SetFocusState.Request()
        req.eye = ['left_eye', 'right_eye']
        req.state = [True, True]
        self.set_focus_state.call_async(req)
        time.sleep(1.0)


def main(args=None):
    """Run main loop."""
    rclpy.init(args=args)

    zoom_controller_service = ZoomControllerService()
    # zoom_controller_service.start_autofocus()
    rclpy.spin(zoom_controller_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
