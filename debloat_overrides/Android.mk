LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := KleeDebloatOverrides
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_SUFFIX := .marker
LOCAL_SRC_FILES := debloat_overrides.marker
LOCAL_UNINSTALLABLE_MODULE := true

# Build-time package filter metadata; this module is not installed.
LOCAL_OVERRIDES_MODULES := \
    FM2 \
    PhotoTable \
    LocalContactsBackup \
    QuickAccessWallet \
    LMOFreeform \
    LMOFreeformSidebar \
    BasicDreams \
    BluetoothMidiService \
    Stk \
    WallpaperBackup \
    CallLogBackup \
    CellBroadcastLegacyApp \
    DeviceDiagnostics \
    LiveWallpapersPicker \
    BtHelper \
    BtHelperAdapter \
    EmergencyInfo \
    Updater \
    OmniJaws \
    Gramophone \
    ANGLE \
    AppSearchAiSealConfig \
    SimAppDialog \
    BackupRestoreConfirmation \
    Datura \
    GameSpace \
    Seedvault
include $(BUILD_PREBUILT)
