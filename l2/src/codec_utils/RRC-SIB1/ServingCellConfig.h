/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_ServingCellConfig_H_
#define	_ServingCellConfig_H_


#include <asn_application.h>

/* Including external dependencies */
#include "BWP-Id.h"
#include <NativeEnumerated.h>
#include "TAG-Id.h"
#include "MeasObjectId.h"
#include <asn_SEQUENCE_OF.h>
#include <constr_SEQUENCE_OF.h>
#include "RateMatchPatternId.h"
#include <constr_SEQUENCE.h>
#include <NativeInteger.h>
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum ServingCellConfig__bwp_InactivityTimer {
	ServingCellConfig__bwp_InactivityTimer_ms2	= 0,
	ServingCellConfig__bwp_InactivityTimer_ms3	= 1,
	ServingCellConfig__bwp_InactivityTimer_ms4	= 2,
	ServingCellConfig__bwp_InactivityTimer_ms5	= 3,
	ServingCellConfig__bwp_InactivityTimer_ms6	= 4,
	ServingCellConfig__bwp_InactivityTimer_ms8	= 5,
	ServingCellConfig__bwp_InactivityTimer_ms10	= 6,
	ServingCellConfig__bwp_InactivityTimer_ms20	= 7,
	ServingCellConfig__bwp_InactivityTimer_ms30	= 8,
	ServingCellConfig__bwp_InactivityTimer_ms40	= 9,
	ServingCellConfig__bwp_InactivityTimer_ms50	= 10,
	ServingCellConfig__bwp_InactivityTimer_ms60	= 11,
	ServingCellConfig__bwp_InactivityTimer_ms80	= 12,
	ServingCellConfig__bwp_InactivityTimer_ms100	= 13,
	ServingCellConfig__bwp_InactivityTimer_ms200	= 14,
	ServingCellConfig__bwp_InactivityTimer_ms300	= 15,
	ServingCellConfig__bwp_InactivityTimer_ms500	= 16,
	ServingCellConfig__bwp_InactivityTimer_ms750	= 17,
	ServingCellConfig__bwp_InactivityTimer_ms1280	= 18,
	ServingCellConfig__bwp_InactivityTimer_ms1920	= 19,
	ServingCellConfig__bwp_InactivityTimer_ms2560	= 20,
	ServingCellConfig__bwp_InactivityTimer_spare10	= 21,
	ServingCellConfig__bwp_InactivityTimer_spare9	= 22,
	ServingCellConfig__bwp_InactivityTimer_spare8	= 23,
	ServingCellConfig__bwp_InactivityTimer_spare7	= 24,
	ServingCellConfig__bwp_InactivityTimer_spare6	= 25,
	ServingCellConfig__bwp_InactivityTimer_spare5	= 26,
	ServingCellConfig__bwp_InactivityTimer_spare4	= 27,
	ServingCellConfig__bwp_InactivityTimer_spare3	= 28,
	ServingCellConfig__bwp_InactivityTimer_spare2	= 29,
	ServingCellConfig__bwp_InactivityTimer_spare1	= 30
} e_ServingCellConfig__bwp_InactivityTimer;
typedef enum ServingCellConfig__sCellDeactivationTimer {
	ServingCellConfig__sCellDeactivationTimer_ms20	= 0,
	ServingCellConfig__sCellDeactivationTimer_ms40	= 1,
	ServingCellConfig__sCellDeactivationTimer_ms80	= 2,
	ServingCellConfig__sCellDeactivationTimer_ms160	= 3,
	ServingCellConfig__sCellDeactivationTimer_ms200	= 4,
	ServingCellConfig__sCellDeactivationTimer_ms240	= 5,
	ServingCellConfig__sCellDeactivationTimer_ms320	= 6,
	ServingCellConfig__sCellDeactivationTimer_ms400	= 7,
	ServingCellConfig__sCellDeactivationTimer_ms480	= 8,
	ServingCellConfig__sCellDeactivationTimer_ms520	= 9,
	ServingCellConfig__sCellDeactivationTimer_ms640	= 10,
	ServingCellConfig__sCellDeactivationTimer_ms720	= 11,
	ServingCellConfig__sCellDeactivationTimer_ms840	= 12,
	ServingCellConfig__sCellDeactivationTimer_ms1280	= 13,
	ServingCellConfig__sCellDeactivationTimer_spare2	= 14,
	ServingCellConfig__sCellDeactivationTimer_spare1	= 15
} e_ServingCellConfig__sCellDeactivationTimer;
typedef enum ServingCellConfig__dummy1 {
	ServingCellConfig__dummy1_enabled	= 0
} e_ServingCellConfig__dummy1;
typedef enum ServingCellConfig__pathlossReferenceLinking {
	ServingCellConfig__pathlossReferenceLinking_spCell	= 0,
	ServingCellConfig__pathlossReferenceLinking_sCell	= 1
} e_ServingCellConfig__pathlossReferenceLinking;
typedef enum ServingCellConfig__ext2__supplementaryUplinkRelease_r16 {
	ServingCellConfig__ext2__supplementaryUplinkRelease_r16_true	= 0
} e_ServingCellConfig__ext2__supplementaryUplinkRelease_r16;
typedef enum ServingCellConfig__ext2__ca_SlotOffset_r16_PR {
	ServingCellConfig__ext2__ca_SlotOffset_r16_PR_NOTHING,	/* No components present */
	ServingCellConfig__ext2__ca_SlotOffset_r16_PR_refSCS15kHz,
	ServingCellConfig__ext2__ca_SlotOffset_r16_PR_refSCS30KHz,
	ServingCellConfig__ext2__ca_SlotOffset_r16_PR_refSCS60KHz,
	ServingCellConfig__ext2__ca_SlotOffset_r16_PR_refSCS120KHz
} ServingCellConfig__ext2__ca_SlotOffset_r16_PR;
typedef enum ServingCellConfig__ext2__csi_RS_ValidationWithDCI_r16 {
	ServingCellConfig__ext2__csi_RS_ValidationWithDCI_r16_enabled	= 0
} e_ServingCellConfig__ext2__csi_RS_ValidationWithDCI_r16;
typedef enum ServingCellConfig__ext2__crs_RateMatch_PerCORESETPoolIndex_r16 {
	ServingCellConfig__ext2__crs_RateMatch_PerCORESETPoolIndex_r16_enabled	= 0
} e_ServingCellConfig__ext2__crs_RateMatch_PerCORESETPoolIndex_r16;
typedef enum ServingCellConfig__ext2__enableTwoDefaultTCI_States_r16 {
	ServingCellConfig__ext2__enableTwoDefaultTCI_States_r16_enabled	= 0
} e_ServingCellConfig__ext2__enableTwoDefaultTCI_States_r16;
typedef enum ServingCellConfig__ext2__enableDefaultTCI_StatePerCoresetPoolIndex_r16 {
	ServingCellConfig__ext2__enableDefaultTCI_StatePerCoresetPoolIndex_r16_enabled	= 0
} e_ServingCellConfig__ext2__enableDefaultTCI_StatePerCoresetPoolIndex_r16;
typedef enum ServingCellConfig__ext2__enableBeamSwitchTiming_r16 {
	ServingCellConfig__ext2__enableBeamSwitchTiming_r16_true	= 0
} e_ServingCellConfig__ext2__enableBeamSwitchTiming_r16;
typedef enum ServingCellConfig__ext2__cbg_TxDiffTBsProcessingType1_r16 {
	ServingCellConfig__ext2__cbg_TxDiffTBsProcessingType1_r16_enabled	= 0
} e_ServingCellConfig__ext2__cbg_TxDiffTBsProcessingType1_r16;
typedef enum ServingCellConfig__ext2__cbg_TxDiffTBsProcessingType2_r16 {
	ServingCellConfig__ext2__cbg_TxDiffTBsProcessingType2_r16_enabled	= 0
} e_ServingCellConfig__ext2__cbg_TxDiffTBsProcessingType2_r16;
typedef enum ServingCellConfig__ext3__directionalCollisionHandling_r16 {
	ServingCellConfig__ext3__directionalCollisionHandling_r16_enabled	= 0
} e_ServingCellConfig__ext3__directionalCollisionHandling_r16;
typedef enum ServingCellConfig__ext4__channelAccessMode2_r17 {
	ServingCellConfig__ext4__channelAccessMode2_r17_enabled	= 0
} e_ServingCellConfig__ext4__channelAccessMode2_r17;
typedef enum ServingCellConfig__ext4__timeDomainHARQ_BundlingType1_r17 {
	ServingCellConfig__ext4__timeDomainHARQ_BundlingType1_r17_enabled	= 0
} e_ServingCellConfig__ext4__timeDomainHARQ_BundlingType1_r17;
typedef enum ServingCellConfig__ext4__nrofHARQ_BundlingGroups_r17 {
	ServingCellConfig__ext4__nrofHARQ_BundlingGroups_r17_n1	= 0,
	ServingCellConfig__ext4__nrofHARQ_BundlingGroups_r17_n2	= 1,
	ServingCellConfig__ext4__nrofHARQ_BundlingGroups_r17_n4	= 2
} e_ServingCellConfig__ext4__nrofHARQ_BundlingGroups_r17;
typedef enum ServingCellConfig__ext4__fdmed_ReceptionMulticast_r17 {
	ServingCellConfig__ext4__fdmed_ReceptionMulticast_r17_true	= 0
} e_ServingCellConfig__ext4__fdmed_ReceptionMulticast_r17;
typedef enum ServingCellConfig__ext4__moreThanOneNackOnlyMode_r17 {
	ServingCellConfig__ext4__moreThanOneNackOnlyMode_r17_mode2	= 0
} e_ServingCellConfig__ext4__moreThanOneNackOnlyMode_r17;
typedef enum ServingCellConfig__ext4__directionalCollisionHandling_DC_r17 {
	ServingCellConfig__ext4__directionalCollisionHandling_DC_r17_enabled	= 0
} e_ServingCellConfig__ext4__directionalCollisionHandling_DC_r17;
typedef enum ServingCellConfig__ext5__lte_NeighCellsCRS_Assumptions_r17 {
	ServingCellConfig__ext5__lte_NeighCellsCRS_Assumptions_r17_false	= 0
} e_ServingCellConfig__ext5__lte_NeighCellsCRS_Assumptions_r17;

/* Forward declarations */
struct TDD_UL_DL_ConfigDedicated;
struct BWP_DownlinkDedicated;
struct UplinkConfig;
struct SetupRelease_PDCCH_ServingCellConfig;
struct SetupRelease_PDSCH_ServingCellConfig;
struct SetupRelease_CSI_MeasConfig;
struct CrossCarrierSchedulingConfig;
struct BWP_Downlink;
struct SetupRelease_RateMatchPatternLTE_CRS;
struct RateMatchPattern;
struct SCS_SpecificCarrier;
struct TDD_UL_DL_ConfigDedicated_IAB_MT_r16;
struct SetupRelease_DormantBWP_Config_r16;
struct SetupRelease_DummyJ;
struct SetupRelease_LTE_CRS_PatternList_r16;
struct IntraCellGuardBandsPerSCS_r16;
struct SetupRelease_ChannelAccessConfig_r16;
struct SetupRelease_NR_DL_PRS_PDC_Info_r17;
struct SetupRelease_SemiStaticChannelAccessConfigUE_r17;
struct SetupRelease_MIMOParam_r17;
struct TCI_ActivatedConfig_r17;
struct SetupRelease_LTE_NeighCellsCRS_AssistInfoList_r17;

/* ServingCellConfig */
typedef struct ServingCellConfig {
	struct TDD_UL_DL_ConfigDedicated	*tdd_UL_DL_ConfigurationDedicated;	/* OPTIONAL */
	struct BWP_DownlinkDedicated	*initialDownlinkBWP;	/* OPTIONAL */
	struct ServingCellConfig__downlinkBWP_ToReleaseList {
		A_SEQUENCE_OF(BWP_Id_t) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *downlinkBWP_ToReleaseList;
	struct ServingCellConfig__downlinkBWP_ToAddModList {
		A_SEQUENCE_OF(struct BWP_Downlink) list;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *downlinkBWP_ToAddModList;
	BWP_Id_t	*firstActiveDownlinkBWP_Id;	/* OPTIONAL */
	long	*bwp_InactivityTimer;	/* OPTIONAL */
	BWP_Id_t	*defaultDownlinkBWP_Id;	/* OPTIONAL */
	struct UplinkConfig	*uplinkConfig;	/* OPTIONAL */
	struct UplinkConfig	*supplementaryUplink;	/* OPTIONAL */
	struct SetupRelease_PDCCH_ServingCellConfig	*pdcch_ServingCellConfig;	/* OPTIONAL */
	struct SetupRelease_PDSCH_ServingCellConfig	*pdsch_ServingCellConfig;	/* OPTIONAL */
	struct SetupRelease_CSI_MeasConfig	*csi_MeasConfig;	/* OPTIONAL */
	long	*sCellDeactivationTimer;	/* OPTIONAL */
	struct CrossCarrierSchedulingConfig	*crossCarrierSchedulingConfig;	/* OPTIONAL */
	TAG_Id_t	 tag_Id;
	long	*dummy1;	/* OPTIONAL */
	long	*pathlossReferenceLinking;	/* OPTIONAL */
	MeasObjectId_t	*servingCellMO;	/* OPTIONAL */
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct ServingCellConfig__ext1 {
		struct SetupRelease_RateMatchPatternLTE_CRS	*lte_CRS_ToMatchAround;	/* OPTIONAL */
		struct ServingCellConfig__ext1__rateMatchPatternToAddModList {
			A_SEQUENCE_OF(struct RateMatchPattern) list;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *rateMatchPatternToAddModList;
		struct ServingCellConfig__ext1__rateMatchPatternToReleaseList {
			A_SEQUENCE_OF(RateMatchPatternId_t) list;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *rateMatchPatternToReleaseList;
		struct ServingCellConfig__ext1__downlinkChannelBW_PerSCS_List {
			A_SEQUENCE_OF(struct SCS_SpecificCarrier) list;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *downlinkChannelBW_PerSCS_List;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	struct ServingCellConfig__ext2 {
		long	*supplementaryUplinkRelease_r16;	/* OPTIONAL */
		struct TDD_UL_DL_ConfigDedicated_IAB_MT_r16	*tdd_UL_DL_ConfigurationDedicated_IAB_MT_r16;	/* OPTIONAL */
		struct SetupRelease_DormantBWP_Config_r16	*dormantBWP_Config_r16;	/* OPTIONAL */
		struct ServingCellConfig__ext2__ca_SlotOffset_r16 {
			ServingCellConfig__ext2__ca_SlotOffset_r16_PR present;
			union ServingCellConfig__ext2__ca_SlotOffset_r16_u {
				long	 refSCS15kHz;
				long	 refSCS30KHz;
				long	 refSCS60KHz;
				long	 refSCS120KHz;
			} choice;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *ca_SlotOffset_r16;
		struct SetupRelease_DummyJ	*dummy2;	/* OPTIONAL */
		struct ServingCellConfig__ext2__intraCellGuardBandsDL_List_r16 {
			A_SEQUENCE_OF(struct IntraCellGuardBandsPerSCS_r16) list;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *intraCellGuardBandsDL_List_r16;
		struct ServingCellConfig__ext2__intraCellGuardBandsUL_List_r16 {
			A_SEQUENCE_OF(struct IntraCellGuardBandsPerSCS_r16) list;
			
			/* Context for parsing across buffer boundaries */
			asn_struct_ctx_t _asn_ctx;
		} *intraCellGuardBandsUL_List_r16;
		long	*csi_RS_ValidationWithDCI_r16;	/* OPTIONAL */
		struct SetupRelease_LTE_CRS_PatternList_r16	*lte_CRS_PatternList1_r16;	/* OPTIONAL */
		struct SetupRelease_LTE_CRS_PatternList_r16	*lte_CRS_PatternList2_r16;	/* OPTIONAL */
		long	*crs_RateMatch_PerCORESETPoolIndex_r16;	/* OPTIONAL */
		long	*enableTwoDefaultTCI_States_r16;	/* OPTIONAL */
		long	*enableDefaultTCI_StatePerCoresetPoolIndex_r16;	/* OPTIONAL */
		long	*enableBeamSwitchTiming_r16;	/* OPTIONAL */
		long	*cbg_TxDiffTBsProcessingType1_r16;	/* OPTIONAL */
		long	*cbg_TxDiffTBsProcessingType2_r16;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext2;
	struct ServingCellConfig__ext3 {
		long	*directionalCollisionHandling_r16;	/* OPTIONAL */
		struct SetupRelease_ChannelAccessConfig_r16	*channelAccessConfig_r16;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext3;
	struct ServingCellConfig__ext4 {
		struct SetupRelease_NR_DL_PRS_PDC_Info_r17	*nr_dl_PRS_PDC_Info_r17;	/* OPTIONAL */
		struct SetupRelease_SemiStaticChannelAccessConfigUE_r17	*semiStaticChannelAccessConfigUE_r17;	/* OPTIONAL */
		struct SetupRelease_MIMOParam_r17	*mimoParam_r17;	/* OPTIONAL */
		long	*channelAccessMode2_r17;	/* OPTIONAL */
		long	*timeDomainHARQ_BundlingType1_r17;	/* OPTIONAL */
		long	*nrofHARQ_BundlingGroups_r17;	/* OPTIONAL */
		long	*fdmed_ReceptionMulticast_r17;	/* OPTIONAL */
		long	*moreThanOneNackOnlyMode_r17;	/* OPTIONAL */
		struct TCI_ActivatedConfig_r17	*tci_ActivatedConfig_r17;	/* OPTIONAL */
		long	*directionalCollisionHandling_DC_r17;	/* OPTIONAL */
		struct SetupRelease_LTE_NeighCellsCRS_AssistInfoList_r17	*lte_NeighCellsCRS_AssistInfoList_r17;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext4;
	struct ServingCellConfig__ext5 {
		long	*lte_NeighCellsCRS_Assumptions_r17;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext5;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} ServingCellConfig_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_bwp_InactivityTimer_9;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_sCellDeactivationTimer_47;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_dummy1_66;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_pathlossReferenceLinking_68;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_supplementaryUplinkRelease_r16_82;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_csi_RS_ValidationWithDCI_r16_96;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_crs_RateMatch_PerCORESETPoolIndex_r16_100;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_enableTwoDefaultTCI_States_r16_102;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_enableDefaultTCI_StatePerCoresetPoolIndex_r16_104;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_enableBeamSwitchTiming_r16_106;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_cbg_TxDiffTBsProcessingType1_r16_108;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_cbg_TxDiffTBsProcessingType2_r16_110;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_directionalCollisionHandling_r16_113;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_channelAccessMode2_r17_120;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_timeDomainHARQ_BundlingType1_r17_122;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_nrofHARQ_BundlingGroups_r17_124;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_fdmed_ReceptionMulticast_r17_128;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_moreThanOneNackOnlyMode_r17_130;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_directionalCollisionHandling_DC_r17_133;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_lte_NeighCellsCRS_Assumptions_r17_137;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_ServingCellConfig;
extern asn_SEQUENCE_specifics_t asn_SPC_ServingCellConfig_specs_1;
extern asn_TYPE_member_t asn_MBR_ServingCellConfig_1[23];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "TDD-UL-DL-ConfigDedicated.h"
#include "BWP-DownlinkDedicated.h"
#include "UplinkConfig.h"
#include "SetupRelease.h"
#include "CrossCarrierSchedulingConfig.h"
#include "BWP-Downlink.h"
#include "RateMatchPattern.h"
#include "SCS-SpecificCarrier.h"
#include "TDD-UL-DL-ConfigDedicated-IAB-MT-r16.h"
#include "IntraCellGuardBandsPerSCS-r16.h"
#include "TCI-ActivatedConfig-r17.h"

#endif	/* _ServingCellConfig_H_ */
#include <asn_internal.h>
