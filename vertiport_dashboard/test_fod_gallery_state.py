from pathlib import Path
from tempfile import TemporaryDirectory
from unittest import TestCase
from unittest.mock import patch

from vertiport_dashboard import fod_gallery_state as gallery


class GalleryStateTests(TestCase):
    def setUp(self):
        directory = TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        state_file = Path(directory.name) / "gallery.json"
        self.enterContext(patch.object(gallery, "GALLERY_STATE_FILE", state_file))

    def test_repeated_resets_keep_old_ids_and_do_not_hide_later_ids(self):
        self.assertEqual(gallery.load_hidden_photos(), set())
        gallery.hide_photos([{"photo": "FOD_same_second_a.jpg"}])
        gallery.hide_photos([{"photo": "FOD_same_second_b.jpg"}])
        self.assertEqual(gallery.load_hidden_photos(), {
            "FOD_same_second_a.jpg", "FOD_same_second_b.jpg",
        })
        self.assertNotIn("FOD_same_second_c.jpg", gallery.load_hidden_photos())

    def test_failed_replace_preserves_previous_state(self):
        gallery.hide_photos([{"photo": "FOD_old.jpg"}])
        with patch.object(Path, "replace", side_effect=PermissionError("read-only")):
            with self.assertRaises(PermissionError):
                gallery.hide_photos([{"photo": "FOD_new.jpg"}])
        self.assertEqual(gallery.load_hidden_photos(), {"FOD_old.jpg"})
