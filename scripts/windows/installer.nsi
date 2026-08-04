Unicode true
RequestExecutionLevel user
SetCompressor /SOLID lzma

!include "MUI2.nsh"
!include "x64.nsh"

!ifndef BUNDLE_DIR
  !error "BUNDLE_DIR is required"
!endif
!ifndef OUTPUT_FILE
  !error "OUTPUT_FILE is required"
!endif
!ifndef PRODUCT_VERSION
  !error "PRODUCT_VERSION is required"
!endif

Name "CAMotics Fast ${PRODUCT_VERSION}"
OutFile "${OUTPUT_FILE}"
InstallDir "$LOCALAPPDATA\Programs\CAMotics Fast"
InstallDirRegKey HKCU "Software\CAMotics Fast" "InstallDir"

VIProductVersion "${PRODUCT_VERSION}.0"
VIAddVersionKey /LANG=1033 "ProductName" "CAMotics Fast"
VIAddVersionKey /LANG=1033 "FileDescription" "CAMotics Fast installer"
VIAddVersionKey /LANG=1033 "FileVersion" "${PRODUCT_VERSION}"
VIAddVersionKey /LANG=1033 "LegalCopyright" "CAMotics contributors and davronthemighty"

!define MUI_ABORTWARNING
!define MUI_FINISHPAGE_RUN "$INSTDIR\camotics.exe"
!insertmacro MUI_PAGE_WELCOME
!insertmacro MUI_PAGE_DIRECTORY
!insertmacro MUI_PAGE_INSTFILES
!insertmacro MUI_PAGE_FINISH
!insertmacro MUI_UNPAGE_CONFIRM
!insertmacro MUI_UNPAGE_INSTFILES
!insertmacro MUI_LANGUAGE "English"

Section "CAMotics Fast" SEC_MAIN
  SetOutPath "$INSTDIR"
  File /r "${BUNDLE_DIR}\*"
  WriteUninstaller "$INSTDIR\Uninstall.exe"
  WriteRegStr HKCU "Software\CAMotics Fast" "InstallDir" "$INSTDIR"
  WriteRegStr HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\CAMotics Fast" "DisplayName" "CAMotics Fast"
  WriteRegStr HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\CAMotics Fast" "DisplayVersion" "${PRODUCT_VERSION}"
  WriteRegStr HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\CAMotics Fast" "Publisher" "davronthemighty"
  WriteRegStr HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\CAMotics Fast" "UninstallString" '"$INSTDIR\Uninstall.exe"'
  CreateDirectory "$SMPROGRAMS\CAMotics Fast"
  CreateShortcut "$SMPROGRAMS\CAMotics Fast\CAMotics Fast.lnk" "$INSTDIR\camotics.exe"
  CreateShortcut "$SMPROGRAMS\CAMotics Fast\Uninstall.lnk" "$INSTDIR\Uninstall.exe"
SectionEnd
Section "Uninstall"
  Delete "$SMPROGRAMS\CAMotics Fast\CAMotics Fast.lnk"
  Delete "$SMPROGRAMS\CAMotics Fast\Uninstall.lnk"
  RMDir "$SMPROGRAMS\CAMotics Fast"
  DeleteRegKey HKCU "Software\Microsoft\Windows\CurrentVersion\Uninstall\CAMotics Fast"
  DeleteRegKey HKCU "Software\CAMotics Fast"
  RMDir /r "$INSTDIR"
SectionEnd
