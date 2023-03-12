class Scene:
    @staticmethod
    def load(path: str) -> "Scene":
        raise NotImplementedError()

    def save(self, path: str):
        raise NotImplementedError()

    @property
    def features(self):
        raise NotImplementedError()
