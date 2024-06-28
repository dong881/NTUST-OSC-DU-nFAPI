/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_CA_ParametersNR_v1730_H_
#define	_CA_ParametersNR_v1730_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <NativeInteger.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum CA_ParametersNR_v1730__dmrs_BundlingPUSCH_RepTypeAPerBC_r17 {
	CA_ParametersNR_v1730__dmrs_BundlingPUSCH_RepTypeAPerBC_r17_supported	= 0
} e_CA_ParametersNR_v1730__dmrs_BundlingPUSCH_RepTypeAPerBC_r17;
typedef enum CA_ParametersNR_v1730__dmrs_BundlingPUSCH_RepTypeBPerBC_r17 {
	CA_ParametersNR_v1730__dmrs_BundlingPUSCH_RepTypeBPerBC_r17_supported	= 0
} e_CA_ParametersNR_v1730__dmrs_BundlingPUSCH_RepTypeBPerBC_r17;
typedef enum CA_ParametersNR_v1730__dmrs_BundlingPUSCH_multiSlotPerBC_r17 {
	CA_ParametersNR_v1730__dmrs_BundlingPUSCH_multiSlotPerBC_r17_supported	= 0
} e_CA_ParametersNR_v1730__dmrs_BundlingPUSCH_multiSlotPerBC_r17;
typedef enum CA_ParametersNR_v1730__dmrs_BundlingPUCCH_RepPerBC_r17 {
	CA_ParametersNR_v1730__dmrs_BundlingPUCCH_RepPerBC_r17_supported	= 0
} e_CA_ParametersNR_v1730__dmrs_BundlingPUCCH_RepPerBC_r17;
typedef enum CA_ParametersNR_v1730__dmrs_BundlingRestartPerBC_r17 {
	CA_ParametersNR_v1730__dmrs_BundlingRestartPerBC_r17_supported	= 0
} e_CA_ParametersNR_v1730__dmrs_BundlingRestartPerBC_r17;
typedef enum CA_ParametersNR_v1730__dmrs_BundlingNonBackToBackTX_PerBC_r17 {
	CA_ParametersNR_v1730__dmrs_BundlingNonBackToBackTX_PerBC_r17_supported	= 0
} e_CA_ParametersNR_v1730__dmrs_BundlingNonBackToBackTX_PerBC_r17;
typedef enum CA_ParametersNR_v1730__stayOnTargetCC_SRS_CarrierSwitch_r17 {
	CA_ParametersNR_v1730__stayOnTargetCC_SRS_CarrierSwitch_r17_supported	= 0
} e_CA_ParametersNR_v1730__stayOnTargetCC_SRS_CarrierSwitch_r17;
typedef enum CA_ParametersNR_v1730__fdm_CodebookForMux_UnicastMulticastHARQ_ACK_r17 {
	CA_ParametersNR_v1730__fdm_CodebookForMux_UnicastMulticastHARQ_ACK_r17_supported	= 0
} e_CA_ParametersNR_v1730__fdm_CodebookForMux_UnicastMulticastHARQ_ACK_r17;
typedef enum CA_ParametersNR_v1730__mode2_TDM_CodebookForMux_UnicastMulticastHARQ_ACK_r17 {
	CA_ParametersNR_v1730__mode2_TDM_CodebookForMux_UnicastMulticastHARQ_ACK_r17_supported	= 0
} e_CA_ParametersNR_v1730__mode2_TDM_CodebookForMux_UnicastMulticastHARQ_ACK_r17;
typedef enum CA_ParametersNR_v1730__mode1_ForType1_CodebookGeneration_r17 {
	CA_ParametersNR_v1730__mode1_ForType1_CodebookGeneration_r17_supported	= 0
} e_CA_ParametersNR_v1730__mode1_ForType1_CodebookGeneration_r17;
typedef enum CA_ParametersNR_v1730__nack_OnlyFeedbackSpecificResourceForSPS_Multicast_r17 {
	CA_ParametersNR_v1730__nack_OnlyFeedbackSpecificResourceForSPS_Multicast_r17_supported	= 0
} e_CA_ParametersNR_v1730__nack_OnlyFeedbackSpecificResourceForSPS_Multicast_r17;
typedef enum CA_ParametersNR_v1730__multiPUCCH_ConfigForMulticast_r17 {
	CA_ParametersNR_v1730__multiPUCCH_ConfigForMulticast_r17_supported	= 0
} e_CA_ParametersNR_v1730__multiPUCCH_ConfigForMulticast_r17;
typedef enum CA_ParametersNR_v1730__pucch_ConfigForSPS_Multicast_r17 {
	CA_ParametersNR_v1730__pucch_ConfigForSPS_Multicast_r17_supported	= 0
} e_CA_ParametersNR_v1730__pucch_ConfigForSPS_Multicast_r17;
typedef enum CA_ParametersNR_v1730__mux_HARQ_ACK_UnicastMulticast_r17 {
	CA_ParametersNR_v1730__mux_HARQ_ACK_UnicastMulticast_r17_supported	= 0
} e_CA_ParametersNR_v1730__mux_HARQ_ACK_UnicastMulticast_r17;

/* CA-ParametersNR-v1730 */
typedef struct CA_ParametersNR_v1730 {
	long	*dmrs_BundlingPUSCH_RepTypeAPerBC_r17;	/* OPTIONAL */
	long	*dmrs_BundlingPUSCH_RepTypeBPerBC_r17;	/* OPTIONAL */
	long	*dmrs_BundlingPUSCH_multiSlotPerBC_r17;	/* OPTIONAL */
	long	*dmrs_BundlingPUCCH_RepPerBC_r17;	/* OPTIONAL */
	long	*dmrs_BundlingRestartPerBC_r17;	/* OPTIONAL */
	long	*dmrs_BundlingNonBackToBackTX_PerBC_r17;	/* OPTIONAL */
	long	*stayOnTargetCC_SRS_CarrierSwitch_r17;	/* OPTIONAL */
	long	*fdm_CodebookForMux_UnicastMulticastHARQ_ACK_r17;	/* OPTIONAL */
	long	*mode2_TDM_CodebookForMux_UnicastMulticastHARQ_ACK_r17;	/* OPTIONAL */
	long	*mode1_ForType1_CodebookGeneration_r17;	/* OPTIONAL */
	long	*nack_OnlyFeedbackSpecificResourceForSPS_Multicast_r17;	/* OPTIONAL */
	long	*multiPUCCH_ConfigForMulticast_r17;	/* OPTIONAL */
	long	*pucch_ConfigForSPS_Multicast_r17;	/* OPTIONAL */
	long	*maxNumberG_RNTI_HARQ_ACK_Codebook_r17;	/* OPTIONAL */
	long	*mux_HARQ_ACK_UnicastMulticast_r17;	/* OPTIONAL */
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} CA_ParametersNR_v1730_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_dmrs_BundlingPUSCH_RepTypeAPerBC_r17_2;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_dmrs_BundlingPUSCH_RepTypeBPerBC_r17_4;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_dmrs_BundlingPUSCH_multiSlotPerBC_r17_6;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_dmrs_BundlingPUCCH_RepPerBC_r17_8;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_dmrs_BundlingRestartPerBC_r17_10;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_dmrs_BundlingNonBackToBackTX_PerBC_r17_12;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_stayOnTargetCC_SRS_CarrierSwitch_r17_14;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_fdm_CodebookForMux_UnicastMulticastHARQ_ACK_r17_16;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_mode2_TDM_CodebookForMux_UnicastMulticastHARQ_ACK_r17_18;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_mode1_ForType1_CodebookGeneration_r17_20;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_nack_OnlyFeedbackSpecificResourceForSPS_Multicast_r17_22;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_multiPUCCH_ConfigForMulticast_r17_24;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_pucch_ConfigForSPS_Multicast_r17_26;	// (Use -fall-defs-global to expose) */
/* extern asn_TYPE_descriptor_t asn_DEF_mux_HARQ_ACK_UnicastMulticast_r17_29;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_CA_ParametersNR_v1730;
extern asn_SEQUENCE_specifics_t asn_SPC_CA_ParametersNR_v1730_specs_1;
extern asn_TYPE_member_t asn_MBR_CA_ParametersNR_v1730_1[15];

#ifdef __cplusplus
}
#endif

#endif	/* _CA_ParametersNR_v1730_H_ */
#include <asn_internal.h>
