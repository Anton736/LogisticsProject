from abc import ABC, abstractmethod
from src.core.entities import Scenario

class BaseDataLoader(ABC):
    @abstractmethod
    def load_scenario(self) -> Scenario:
        """Метод должен прочитать источник и вернуть полностью собранный Scenario"""
        pass