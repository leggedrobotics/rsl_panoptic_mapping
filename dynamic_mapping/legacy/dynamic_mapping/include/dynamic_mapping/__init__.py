import logging

logging.basicConfig(level=logging.INFO)
logging.getLogger("numexpr").setLevel(logging.WARNING)
