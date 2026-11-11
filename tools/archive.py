"""Background export of a closed round. No radio or Tk access."""
import json
from PIL import Image


def save_round(run, session, result):
    Image.frombytes('RGB', run.size, run.source).save(session / 'source.png')
    for name, image in (('received.png', run.receiver.image()),
                        ('missing_mask.png', run.receiver.missing_mask()),
                        ('error_map.png', run.error_mask())):
        if image is not None:
            image.save(session / name)
    (session / 'received.rgb').write_bytes(bytes(run.receiver.pixels))
    (session / 'summary.json').write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding='utf-8')
