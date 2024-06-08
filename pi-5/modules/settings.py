from json import load
from path import Path

SETTINGS_PATH = Path(__file__).parent.parent + "\\settings.json"

def get_setting(name: str) -> str | int | None:
    with open(SETTINGS_PATH, "r") as f:
        settings = load(f)
        f.close()

    if name not in settings:
        return None

    return settings[name]
    

if __name__ == "__main__":
    print(get_setting("ipv4"))