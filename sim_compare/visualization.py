from __future__ import annotations

import weakref


class CarlaCameraDisplay:
    def __init__(self, pygame_module, actor, width: int, height: int, gamma: float):
        import numpy as np  # pylint: disable=import-outside-toplevel
        import carla  # pylint: disable=import-outside-toplevel

        self.np = np
        self.pg = pygame_module
        self.carla = carla
        self.width = width
        self.height = height
        self.display = self.pg.display.set_mode(
            (width, height), self.pg.HWSURFACE | self.pg.DOUBLEBUF
        )
        self.display.fill((0, 0, 0))
        self.pg.display.flip()
        self.font = self.pg.font.Font(self.pg.font.get_default_font(), 18)
        self.surface = None
        self.sensor = None
        self._spawn_camera(actor, gamma)

    def _spawn_camera(self, actor, gamma: float) -> None:
        world = actor.get_world()
        blueprint = world.get_blueprint_library().find("sensor.camera.rgb")
        blueprint.set_attribute("image_size_x", str(self.width))
        blueprint.set_attribute("image_size_y", str(self.height))
        if blueprint.has_attribute("gamma"):
            blueprint.set_attribute("gamma", str(gamma))

        extent = actor.bounding_box.extent
        self.sensor = world.spawn_actor(
            blueprint,
            self.carla.Transform(
                self.carla.Location(x=-2.0 * extent.x - 1.0, z=2.0 * extent.z + 0.5),
                self.carla.Rotation(pitch=8.0),
            ),
            attach_to=actor,
            attachment_type=self.carla.AttachmentType.SpringArmGhost,
        )
        weak_self = weakref.ref(self)
        self.sensor.listen(lambda image: CarlaCameraDisplay._parse_image(weak_self, image))

    @staticmethod
    def _parse_image(weak_self, image) -> None:
        self = weak_self()
        if self is None:
            return
        array = self.np.frombuffer(image.raw_data, dtype=self.np.dtype("uint8"))
        array = self.np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]
        array = array[:, :, ::-1]
        self.surface = self.pg.surfarray.make_surface(array.swapaxes(0, 1))

    def render(self, overlay_lines: list[str]) -> None:
        if self.surface is not None:
            self.display.blit(self.surface, (0, 0))
        else:
            self.display.fill((0, 0, 0))

        overlay = self.pg.Surface((480, 24 * (len(overlay_lines) + 1)))
        overlay.set_alpha(110)
        overlay.fill((0, 0, 0))
        self.display.blit(overlay, (8, 8))
        y = 14
        for line in overlay_lines:
            text = self.font.render(line, True, (255, 255, 255))
            self.display.blit(text, (16, y))
            y += 22
        self.pg.display.flip()

    def destroy(self) -> None:
        if self.sensor is not None:
            self.sensor.stop()
            self.sensor.destroy()
            self.sensor = None
