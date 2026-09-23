package com.lexonblackzz.vibestation;

import android.app.Activity;
import android.content.ContentResolver;
import android.content.Intent;
import android.database.Cursor;
import android.net.Uri;
import android.os.Bundle;
import android.provider.DocumentsContract;
import android.provider.OpenableColumns;

import org.libsdl.app.SDLActivity;

import java.io.File;
import java.io.FileOutputStream;
import java.io.InputStream;

public final class VibeStationActivity extends SDLActivity {
    private static final int REQUEST_BASE = 5200;
    private static final int PICK_BIOS = 1;
    private static final int PICK_GAME = 2;
    private static final int PICK_ROM_DIRECTORY = 3;

    private static native void nativeOnPickerResult(
            int kind, boolean cancelled, String path, String displayName);

    public void requestVibeStationPicker(int kind) {
        final Intent intent;
        if (kind == PICK_ROM_DIRECTORY) {
            intent = new Intent(Intent.ACTION_OPEN_DOCUMENT_TREE);
            intent.addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION |
                    Intent.FLAG_GRANT_PERSISTABLE_URI_PERMISSION);
        } else {
            intent = new Intent(Intent.ACTION_OPEN_DOCUMENT);
            intent.addCategory(Intent.CATEGORY_OPENABLE);
            intent.setType("*/*");
            intent.addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION |
                    Intent.FLAG_GRANT_PERSISTABLE_URI_PERMISSION);
        }
        startActivityForResult(intent, REQUEST_BASE + kind);
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        final int kind = requestCode - REQUEST_BASE;
        if (kind < PICK_BIOS || kind > PICK_ROM_DIRECTORY) {
            super.onActivityResult(requestCode, resultCode, data);
            return;
        }

        if (resultCode != Activity.RESULT_OK || data == null || data.getData() == null) {
            nativeOnPickerResult(kind, true, "", "");
            return;
        }

        final Uri uri = data.getData();
        final int flags = data.getFlags() &
                (Intent.FLAG_GRANT_READ_URI_PERMISSION |
                 Intent.FLAG_GRANT_WRITE_URI_PERMISSION);
        try {
            getContentResolver().takePersistableUriPermission(
                    uri, flags & Intent.FLAG_GRANT_READ_URI_PERMISSION);
        } catch (Exception ignored) {
            // Some document providers do not offer persistable permissions.
        }

        if (kind == PICK_ROM_DIRECTORY) {
            importRomTreeAsync(uri);
            return;
        }

        try {
            final String displayName = queryDisplayName(uri);
            if (kind == PICK_BIOS) {
                final File biosDir = new File(getFilesDir(), "bios");
                if (!biosDir.exists() && !biosDir.mkdirs()) {
                    throw new IllegalStateException("Could not create BIOS directory");
                }
                final File output = new File(biosDir, safeName(displayName));
                try (InputStream in = getContentResolver().openInputStream(uri);
                     FileOutputStream out = new FileOutputStream(output)) {
                    if (in == null) {
                        throw new IllegalStateException("Could not open BIOS");
                    }
                    final byte[] buffer = new byte[1024 * 1024];
                    int read;
                    while ((read = in.read(buffer)) > 0) {
                        out.write(buffer, 0, read);
                    }
                }
                nativeOnPickerResult(kind, false,
                        output.getAbsolutePath(), displayName);
                return;
            }

            final android.os.ParcelFileDescriptor pfd =
                    getContentResolver().openFileDescriptor(uri, "r");
            if (pfd == null) {
                nativeOnPickerResult(kind, true, "", "");
                return;
            }
            final int fd = pfd.detachFd();
            final String path = "/proc/self/fd/" + fd;
            nativeOnPickerResult(kind, false, path, displayName);
        } catch (Exception e) {
            nativeOnPickerResult(kind, true, "", "");
        }
    }

    private String queryDisplayName(Uri uri) {
        Cursor cursor = null;
        try {
            cursor = getContentResolver().query(
                    uri, new String[]{OpenableColumns.DISPLAY_NAME},
                    null, null, null);
            if (cursor != null && cursor.moveToFirst()) {
                final int index = cursor.getColumnIndex(OpenableColumns.DISPLAY_NAME);
                if (index >= 0) {
                    final String name = cursor.getString(index);
                    if (name != null && !name.isEmpty()) {
                        return name;
                    }
                }
            }
        } catch (Exception ignored) {
        } finally {
            if (cursor != null) {
                cursor.close();
            }
        }
        return "selected-file";
    }

    private static String safeName(String value) {
        if (value == null || value.isEmpty()) {
            return "item";
        }
        return value.replaceAll("[\\\\/:*?\"<>|]", "_");
    }

    private void importRomTreeAsync(final Uri treeUri) {
        new Thread(() -> {
            try {
                final File external = getExternalFilesDir("roms");
                final File base = external != null
                        ? external
                        : new File(getFilesDir(), "roms");
                if (!base.exists() && !base.mkdirs()) {
                    throw new IllegalStateException("Could not create ROM import directory");
                }

                final String rootId = DocumentsContract.getTreeDocumentId(treeUri);
                final String rootName = safeName(
                        rootId.substring(rootId.lastIndexOf(':') + 1));
                final File destination = new File(base,
                        rootName.isEmpty() ? "Imported" : rootName);
                if (!destination.exists() && !destination.mkdirs()) {
                    throw new IllegalStateException("Could not create imported ROM directory");
                }

                copyTree(treeUri, rootId, destination);
                runOnUiThread(() -> nativeOnPickerResult(
                        PICK_ROM_DIRECTORY, false,
                        destination.getAbsolutePath(), destination.getName()));
            } catch (Exception e) {
                runOnUiThread(() -> nativeOnPickerResult(
                        PICK_ROM_DIRECTORY, true, "", ""));
            }
        }, "VibeStation-ROM-Import").start();
    }

    private void copyTree(Uri treeUri, String parentDocumentId, File destination)
            throws Exception {
        final ContentResolver resolver = getContentResolver();
        final Uri childrenUri = DocumentsContract.buildChildDocumentsUriUsingTree(
                treeUri, parentDocumentId);
        final String[] projection = new String[]{
                DocumentsContract.Document.COLUMN_DOCUMENT_ID,
                DocumentsContract.Document.COLUMN_DISPLAY_NAME,
                DocumentsContract.Document.COLUMN_MIME_TYPE
        };

        try (Cursor cursor = resolver.query(childrenUri, projection,
                null, null, null)) {
            if (cursor == null) {
                return;
            }

            final int idIndex = cursor.getColumnIndex(
                    DocumentsContract.Document.COLUMN_DOCUMENT_ID);
            final int nameIndex = cursor.getColumnIndex(
                    DocumentsContract.Document.COLUMN_DISPLAY_NAME);
            final int mimeIndex = cursor.getColumnIndex(
                    DocumentsContract.Document.COLUMN_MIME_TYPE);

            while (cursor.moveToNext()) {
                final String documentId = cursor.getString(idIndex);
                final String displayName = safeName(cursor.getString(nameIndex));
                final String mime = cursor.getString(mimeIndex);
                if (DocumentsContract.Document.MIME_TYPE_DIR.equals(mime)) {
                    final File childDir = new File(destination, displayName);
                    if (!childDir.exists() && !childDir.mkdirs()) {
                        continue;
                    }
                    copyTree(treeUri, documentId, childDir);
                    continue;
                }

                final String lower = displayName.toLowerCase();
                if (!lower.endsWith(".bin") && !lower.endsWith(".cue")) {
                    continue;
                }

                final Uri documentUri =
                        DocumentsContract.buildDocumentUriUsingTree(treeUri, documentId);
                final File output = new File(destination, displayName);
                try (InputStream in = resolver.openInputStream(documentUri);
                     FileOutputStream out = new FileOutputStream(output)) {
                    if (in == null) {
                        continue;
                    }
                    final byte[] buffer = new byte[1024 * 1024];
                    int read;
                    while ((read = in.read(buffer)) > 0) {
                        out.write(buffer, 0, read);
                    }
                }
            }
        }
    }
}
