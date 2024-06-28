/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#ifndef	_RA_Report_r16_H_
#define	_RA_Report_r16_H_


#include <asn_application.h>

/* Including external dependencies */
#include <NativeEnumerated.h>
#include <constr_CHOICE.h>
#include <constr_SEQUENCE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum RA_Report_r16__cellId_r16_PR {
	RA_Report_r16__cellId_r16_PR_NOTHING,	/* No components present */
	RA_Report_r16__cellId_r16_PR_cellGlobalId_r16,
	RA_Report_r16__cellId_r16_PR_pci_arfcn_r16
} RA_Report_r16__cellId_r16_PR;
typedef enum RA_Report_r16__raPurpose_r16 {
	RA_Report_r16__raPurpose_r16_accessRelated	= 0,
	RA_Report_r16__raPurpose_r16_beamFailureRecovery	= 1,
	RA_Report_r16__raPurpose_r16_reconfigurationWithSync	= 2,
	RA_Report_r16__raPurpose_r16_ulUnSynchronized	= 3,
	RA_Report_r16__raPurpose_r16_schedulingRequestFailure	= 4,
	RA_Report_r16__raPurpose_r16_noPUCCHResourceAvailable	= 5,
	RA_Report_r16__raPurpose_r16_requestForOtherSI	= 6,
	RA_Report_r16__raPurpose_r16_msg3RequestForOtherSI_r17	= 7,
	RA_Report_r16__raPurpose_r16_spare8	= 8,
	RA_Report_r16__raPurpose_r16_spare7	= 9,
	RA_Report_r16__raPurpose_r16_spare6	= 10,
	RA_Report_r16__raPurpose_r16_spare5	= 11,
	RA_Report_r16__raPurpose_r16_spare4	= 12,
	RA_Report_r16__raPurpose_r16_spare3	= 13,
	RA_Report_r16__raPurpose_r16_spare2	= 14,
	RA_Report_r16__raPurpose_r16_spare1	= 15
} e_RA_Report_r16__raPurpose_r16;

/* Forward declarations */
struct RA_InformationCommon_r16;
struct CGI_Info_Logging_r16;
struct PCI_ARFCN_NR_r16;

/* RA-Report-r16 */
typedef struct RA_Report_r16 {
	struct RA_Report_r16__cellId_r16 {
		RA_Report_r16__cellId_r16_PR present;
		union RA_Report_r16__cellId_r16_u {
			struct CGI_Info_Logging_r16	*cellGlobalId_r16;
			struct PCI_ARFCN_NR_r16	*pci_arfcn_r16;
		} choice;
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} cellId_r16;
	struct RA_InformationCommon_r16	*ra_InformationCommon_r16;	/* OPTIONAL */
	long	 raPurpose_r16;
	/*
	 * This type is extensible,
	 * possible extensions are below.
	 */
	struct RA_Report_r16__ext1 {
		struct CGI_Info_Logging_r16	*spCellID_r17;	/* OPTIONAL */
		
		/* Context for parsing across buffer boundaries */
		asn_struct_ctx_t _asn_ctx;
	} *ext1;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} RA_Report_r16_t;

/* Implementation */
/* extern asn_TYPE_descriptor_t asn_DEF_raPurpose_r16_6;	// (Use -fall-defs-global to expose) */
extern asn_TYPE_descriptor_t asn_DEF_RA_Report_r16;
extern asn_SEQUENCE_specifics_t asn_SPC_RA_Report_r16_specs_1;
extern asn_TYPE_member_t asn_MBR_RA_Report_r16_1[4];

#ifdef __cplusplus
}
#endif

/* Referred external types */
#include "RA-InformationCommon-r16.h"
#include "CGI-Info-Logging-r16.h"
#include "PCI-ARFCN-NR-r16.h"

#endif	/* _RA_Report_r16_H_ */
#include <asn_internal.h>
