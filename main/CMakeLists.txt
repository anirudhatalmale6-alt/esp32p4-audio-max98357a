# Collect all WAV files in the audio/ directory for embedding
file(GLOB AUDIO_FILES "${CMAKE_CURRENT_SOURCE_DIR}/audio/*.wav")

idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    EMBED_FILES ${AUDIO_FILES}
    REQUIRES esp_driver_i2s
)
