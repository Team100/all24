from typing import Protocol

from app.camera.interpreter_protocol import Interpreter
from app.config.identity import Identity
from app.camera.camera_protocol import Camera
from app.dashboard.display import Display
from app.network.real_network import Network
from app.localization.note_detector import NoteDetector
from app.localization.tag_detector import TagDetector

class interpreter_factory():
    
    @staticmethod
    def get(self, 
            identity: Identity, 
            cam: Camera,
            camera_num: int,
            display: Display,
            network: Network,) -> Interpreter:
        match identity: 
            case (Identity.GLOBAL_GAME_PIECE
                  |Identity.GAME_PIECE):
                return NoteDetector(identity, cam, camera_num, display, network)
            case (Identity.RIGHTAMP
                | Identity.LEFTAMP
                | Identity.SHOOTER
                | Identity.GLOBAL_LEFT
                | Identity.GLOBAL_RIGHT
                | Identity.DEV):
                return TagDetector(identity, cam, camera_num, display, network)







