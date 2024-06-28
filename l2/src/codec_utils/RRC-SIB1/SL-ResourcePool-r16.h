/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_SL_ResourcePool_r16_H_
#define	_SL_ResourcePool_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <NativeInteger.h>
#include "FilterCoefficient.h"
#include <constr_SEQUENCE.h>
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include <BIT_STRING.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SL_ResourcePool_r16__sl_SubchannelSize_r16 {
	SL_ResourcePool_r16__sl_SubchannelSize_r16_n10	= 0,
	SL_ResourcePool_r16__sl_SubchannelSize_r16_n12	= 1,
	SL_ResourcePool_r16__sl_SubchannelSize_r16_n15	= 2,
	SL_ResourcePool_r16__sl_SubchannelSize_r16_n20	= 3,
	SL_ResourcePool_r16__sl_SubchannelSize_r16_n25	= 4,
	SL_ResourcePool_r16__sl_SubchannelSize_r16_n50	= 5,
	SL_ResourcePool_r16__sl_SubchannelSize_r16_n75	= 6,
	SL_ResourcePool_r16__sl_SubchannelSize_r16_n100	= 7
} e_SL_ResourcePool_r16__sl_SubchannelSize_r16;
typedef enum SL_ResourcePool_r16__sl_Additional_MCS_Table_r16 {
	SL_ResourcePool_r16__sl_Additional_MCS_Table_r16_qam256	= 0,
	SL_ResourcePool_r16__sl_Additional_MCS_Table_r16_qam64LowSE	= 1,
	SL_ResourcePool_r16__sl_Additional_MCS_Table_r16_qam256_qam64LowSE	= 2
} e_SL_ResourcePool_r16__sl_Additional_MCS_Table_r16;
typedef enum SL_ResourcePool_r16__sl_TimeWindowSizeCBR_r16 {
	SL_ResourcePool_r16__sl_TimeWindowSizeCBR_r16_ms100	= 0,
	SL_ResourcePool_r16__sl_TimeWindowSizeCBR_r16_slot100	= 1
} e_SL_ResourcePool_r16__sl_TimeWindowSizeCBR_r16;
typedef enum SL_ResourcePool_r16__sl_TimeWindowSizeCR_r16 {
	SL_ResourcePool_r16__sl_TimeWindowSizeCR_r16_ms1000	= 0,
	SL_ResourcePool_r16__sl_TimeWindowSizeCR_r16_slot1000	= 1
} e_SL_ResourcePool_r16__sl_TimeWindowSizeCR_r16;
typedef enum SL_ResourcePool_r16__sl_PreemptionEnable_r16 {
	SL_ResourcePool_r16__sl_PreemptionEnable_r16_enabled	= 0,
	SL_ResourcePool_r16__sl_PreemptionEnable_r16_pl1	= 1,
	SL_ResourcePool_r16__sl_PreemptionEnable_r16_pl2	= 2,
	SL_ResourcePool_r16__sl_PreemptionEnable_r16_pl3	= 3,
	SL_ResourcePool_r16__sl_PreemptionEnable_r16_pl4	= 4,
	SL_ResourcePool_r16__sl_PreemptionEnable_r16_pl5	= 5,
	SL_ResourcePool_r16__sl_PreemptionEnable_r16_pl6	= 6,
	SL_ResourcePool_r16__sl_PreemptionEnable_r16_pl7	= 7,
	SL_ResourcePool_r16__sl_PreemptionEnable_r16_pl8	= 8
} e_SL_ResourcePool_r16__sl_PreemptionEnable_r16;
typedef enum SL_ResourcePool_r16__sl_X_Overhead_r16 {
	SL_ResourcePool_r16__sl_X_Overhead_r16_n0	= 0,
	SL_ResourcePool_r16__sl_X_Overhead_r16_n3	= 1,
	SL_ResourcePool_r16__sl_X_Overhead_r16_n6	= 2,
	SL_ResourcePool_r16__sl_X_Overhead_r16_n9	= 3
} e_SL_ResourcePool_r16__sl_X_Overhead_r16;

/* Forward declarations */
struct SetupRelease_SL_PSCCH_Config_r16;
struct SetupRelease_SL_PSSCH_Config_r16;
struct SetupRelease_SL_PSFCH_Config_r16;
struct SL_SyncAllowed_r16;
struct SL_PTRS_Config_r16;
struct SL_UE_SelectedConfigRP_r16;
struct SL_PowerControl_r16;
struct SL_TxPercentageList_r16;
struct SL_MinMaxMCS_List_r16;
struct TDD_UL_DL_ConfigCommon;
struct SL_ZoneConfigMCR_r16;
struct SetupRelease_SL_PBPS_CPS_Config_r17;
struct SetupRelease_SL_InterUE_CoordinationConfig_r17;

/* SL-ResourcePool-r16 */
typedef struct SL_ResourcePool_r16 {
	struct SetupRelease_SL_PSCCH_Config_r16	*sl_PSCCH_Config_r16;	/* OPTIONAL */
	struct SetupRelease_SL_PSSCH_Config_r16	*sl_PSSCH_Config_r16;	/* OPTIONAL */
	struct SetupRelease_SL_PSFCH_Config_r16	*sl_PSFCH_Config_r16;	/* OPTIONAL */
	struct SL_SyncAllowed_r16	*sl_SyncAllowed_r16;	/* OPTIONAL */
	long	*sl_SubchannelSize_r16;	/* OPTIONAL */
	long	*dummy;	/* OPTIONAL */
	long	*sl_StartRB_Subchannel_r16;	/* OPTIONAL */
	long	*sl_NumSubchannel_r16;	/* OPTIONAL */
	long	*sl_Additional_MCS_Table_r16;	/* OPTIONAL */
	long	*sl_ThreshS_RSSI_CBR_r16;	/* OPTIONAL */
	long	*sl_TimeWindowSizeCBR_r16;	/* OPTIONAL */
	long	*sl_TimeWindowSizeCR_r16;	/* OPTIONAL */
	struct SL_PTRS_Config_r16	*sl_PTRS_Config_r16;	/* OPTIONAL */
	struct SL_UE_SelectedConfigRP_r16	*sl_UE_SelectedConfigRP_r16;	/* OPTIONAL */
	struct SL_ResourcePool_r16__sl_RxParametersNcell_r16 {
		struct TDD_UL_DL_ConfigCommon	*sl_TDD_Configuration_r16;	/* OPTIONAL */
		long	 sl_SyncConfigIndex_r16;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *sl_RxParametersNcell_r16;
	struct SL_ResourcePool_r16__sl_ZoneConfigMCR_List_r16 {
		A_SEQUENCE_OF(struct SL_ZoneConfigMCR_r16) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *sl_ZoneConfigMCR_List_r16;
	FilterCoefficient_t	*sl_FilterCoefficient_r16;	/* OPTIONAL */
	long	*sl_RB_Number_r16;	/* OPTIONAL */
	long	*sl_PreemptionEnable_r16;	/* OPTIONAL */
	long	*sl_PriorityThreshold_UL_URLLC_r16;	/* OPTIONAL */
	long	*sl_PriorityThreshold_r16;	/* OPTIONAL */
	long	*sl_X_Overhead_r16;	/* OPTIONAL */
	struct SL_PowerControl_r16	*sl_PowerControl_r16;	/* OPTIONAL */
	struct SL_TxPercentageList_r16	*sl_TxPercentageList_r16;	/* OPTIONAL */
	struct SL_MinMaxMCS_List_r16	*sl_MinMaxMCS_List_r16;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct SL_ResourcePool_r16__ext1 {
		BIT_STRING_t	*sl_TimeResource_r16;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	struct SL_ResourcePool_r16__ext2 {
		struct SetupRelease_SL_PBPS_CPS_Config_r17	*sl_PBPS_CPS_Config_r17;	/* OPTIONAL */
		struct SetupRelease_SL_InterUE_CoordinationConfig_r17	*sl_InterUE_CoordinationConfig_r17;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext2;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SL_ResourcePool_r16_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_sl_SubchannelSize_r16_6;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_sl_Additional_MCS_Table_r16_18;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_sl_TimeWindowSizeCBR_r16_23;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_sl_TimeWindowSizeCR_r16_26;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_sl_PreemptionEnable_r16_38;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_sl_X_Overhead_r16_50;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_SL_ResourcePool_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_SL_ResourcePool_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_SL_ResourcePool_r16_1[27];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "SetupRelease.h"
#include "SL-SyncAllowed-r16.h"
#include "SL-PTRS-Config-r16.h"
#include "SL-UE-SelectedConfigRP-r16.h"
#include "SL-PowerControl-r16.h"
#include "SL-TxPercentageList-r16.h"
#include "SL-MinMaxMCS-List-r16.h"
#include "TDD-UL-DL-ConfigCommon.h"
#include "SL-ZoneConfigMCR-r16.h"

#endif	/* _SL_ResourcePool_r16_H_ */
#include <asn_internal.h>
