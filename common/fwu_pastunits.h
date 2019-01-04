    /** Note! This file should be included by pastunits.h to include this
     *  module's units in the past unit list. This means nothing else than
     *  the below definitions may be included.
     */

    /** Firmware upgrade state (uint32_t) */
    fwu_state = 100,
    /** Address in serial flash where the download is taking place (uint32_t) */
    fwu_download_address,
    /** Address in serial flash where the backup is stored (uint32_t) */
    fwu_backup_address,
