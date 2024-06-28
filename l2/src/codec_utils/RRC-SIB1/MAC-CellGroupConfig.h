/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_MAC_CellGroupConfig_H_
#define	_MAC_CellGroupConfig_H_


#include <asn_application.h>

/* Including external dependencies */
#include <BOOLEAN.h>
#include <constr_SEQUENCE.h>
#include <NativeEnumerated.h>
#include "SchedulingRequestId.h"
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include "MBS-RNTI-SpecificConfigId-r17.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum MAC_CellGroupConfig__ext2__usePreBSR_r16 {
	MAC_CellGroupConfig__ext2__usePreBSR_r16_true	= 0
} e_MAC_CellGroupConfig__ext2__usePreBSR_r16;
typedef enum MAC_CellGroupConfig__ext2__lch_BasedPrioritization_r16 {
	MAC_CellGroupConfig__ext2__lch_BasedPrioritization_r16_enabled	= 0
} e_MAC_CellGroupConfig__ext2__lch_BasedPrioritization_r16;
typedef enum MAC_CellGroupConfig__ext3__enhancedSkipUplinkTxDynamic_r16 {
	MAC_CellGroupConfig__ext3__enhancedSkipUplinkTxDynamic_r16_true	= 0
} e_MAC_CellGroupConfig__ext3__enhancedSkipUplinkTxDynamic_r16;
typedef enum MAC_CellGroupConfig__ext3__enhancedSkipUplinkTxConfigured_r16 {
	MAC_CellGroupConfig__ext3__enhancedSkipUplinkTxConfigured_r16_true	= 0
} e_MAC_CellGroupConfig__ext3__enhancedSkipUplinkTxConfigured_r16;
typedef enum MAC_CellGroupConfig__ext4__intraCG_Prioritization_r17 {
	MAC_CellGroupConfig__ext4__intraCG_Prioritization_r17_enabled	= 0
} e_MAC_CellGroupConfig__ext4__intraCG_Prioritization_r17;
typedef enum MAC_CellGroupConfig__ext5__drx_LastTransmissionUL_r17 {
	MAC_CellGroupConfig__ext5__drx_LastTransmissionUL_r17_enabled	= 0
} e_MAC_CellGroupConfig__ext5__drx_LastTransmissionUL_r17;

/* Forward declarations */
struct SetupRelease_DRX_Config;
struct SchedulingRequestConfig;
struct BSR_Config;
struct TAG_Config;
struct SetupRelease_PHR_Config;
struct SetupRelease_DataInactivityTimer;
struct SetupRelease_DRX_ConfigSecondaryGroup_r16;
struct SetupRelease_DRX_ConfigSL_r17;
struct SetupRelease_DRX_ConfigExt_v1700;
struct SchedulingRequestConfig_v1700;
struct SetupRelease_TAR_Config_r17;
struct MBS_RNTI_SpecificConfig_r17;

/* MAC-CellGroupConfig */
typedef struct MAC_CellGroupConfig {
	struct SetupRelease_DRX_Config	*drx_Config;	/* OPTIONAL */
	struct SchedulingRequestConfig	*schedulingRequestConfig;	/* OPTIONAL */
	struct BSR_Config	*bsr_Config;	/* OPTIONAL */
	struct TAG_Config	*tag_Config;	/* OPTIONAL */
	struct SetupRelease_PHR_Config	*phr_Config;	/* OPTIONAL */
	BOOLEAN_t	 skipUplinkTxDynamic;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct MAC_CellGroupConfig__ext1 {
		BOOLEAN_t	*csi_Mask;	/* OPTIONAL */
		struct SetupRelease_DataInactivityTimer	*dataInactivityTimer;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	struct MAC_CellGroupConfig__ext2 {
		long	*usePreBSR_r16;	/* OPTIONAL */
		SchedulingRequestId_t	*schedulingRequestID_LBT_SCell_r16;	/* OPTIONAL */
		long	*lch_BasedPrioritization_r16;	/* OPTIONAL */
		SchedulingRequestId_t	*schedulingRequestID_BFR_SCell_r16;	/* OPTIONAL */
		struct SetupRelease_DRX_ConfigSecondaryGroup_r16	*drx_ConfigSecondaryGroup_r16;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext2;
	struct MAC_CellGroupConfig__ext3 {
		long	*enhancedSkipUplinkTxDynamic_r16;	/* OPTIONAL */
		long	*enhancedSkipUplinkTxConfigured_r16;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext3;
	struct MAC_CellGroupConfig__ext4 {
		long	*intraCG_Prioritization_r17;	/* OPTIONAL */
		struct SetupRelease_DRX_ConfigSL_r17	*drx_ConfigSL_r17;	/* OPTIONAL */
		struct SetupRelease_DRX_ConfigExt_v1700	*drx_ConfigExt_v1700;	/* OPTIONAL */
		SchedulingRequestId_t	*schedulingRequestID_BFR_r17;	/* OPTIONAL */
		SchedulingRequestId_t	*schedulingRequestID_BFR2_r17;	/* OPTIONAL */
		struct SchedulingRequestConfig_v1700	*schedulingRequestConfig_v1700;	/* OPTIONAL */
		struct SetupRelease_TAR_Config_r17	*tar_Config_r17;	/* OPTIONAL */
		struct MAC_CellGroupConfig__ext4__g_RNTI_ConfigToAddModList_r17 {
			A_SEQUENCE_OF(struct MBS_RNTI_SpecificConfig_r17) list;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *g_RNTI_ConfigToAddModList_r17;
		struct MAC_CellGroupConfig__ext4__g_RNTI_ConfigToReleaseList_r17 {
			A_SEQUENCE_OF(MBS_RNTI_SpecificConfigId_r17_t) list;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *g_RNTI_ConfigToReleaseList_r17;
		struct MAC_CellGroupConfig__ext4__g_CS_RNTI_ConfigToAddModList_r17 {
			A_SEQUENCE_OF(struct MBS_RNTI_SpecificConfig_r17) list;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *g_CS_RNTI_ConfigToAddModList_r17;
		struct MAC_CellGroupConfig__ext4__g_CS_RNTI_ConfigToReleaseList_r17 {
			A_SEQUENCE_OF(MBS_RNTI_SpecificConfigId_r17_t) list;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *g_CS_RNTI_ConfigToReleaseList_r17;
		BOOLEAN_t	*allowCSI_SRS_Tx_MulticastDRX_Active_r17;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext4;
	struct MAC_CellGroupConfig__ext5 {
		SchedulingRequestId_t	*schedulingRequestID_PosMG_Request_r17;	/* OPTIONAL */
		long	*drx_LastTransmissionUL_r17;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext5;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} MAC_CellGroupConfig_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_usePreBSR_r16_13;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_lch_BasedPrioritization_r16_16;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_enhancedSkipUplinkTxDynamic_r16_21;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_enhancedSkipUplinkTxConfigured_r16_23;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_intraCG_Prioritization_r17_26;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_drx_LastTransmissionUL_r17_45;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_MAC_CellGroupConfig;
extern asn_SEQUENCE_specifics_t asn_SPC_MAC_CellGroupConfig_specs_1;
extern asn_TYPE_member_t asn_MBR_MAC_CellGroupConfig_1[11];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "SetupRelease.h"
#include "SchedulingRequestConfig.h"
#include "BSR-Config.h"
#include "TAG-Config.h"
#include "SchedulingRequestConfig-v1700.h"
#include "MBS-RNTI-SpecificConfig-r17.h"

#endif	/* _MAC_CellGroupConfig_H_ */
#include <asn_internal.h>
