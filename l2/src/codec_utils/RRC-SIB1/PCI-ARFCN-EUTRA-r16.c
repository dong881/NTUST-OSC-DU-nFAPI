/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "PCI-ARFCN-EUTRA-r16.h"

asn_TYPE_member_t asn_MBR_PCI_ARFCN_EUTRA_r16_1[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct PCI_ARFCN_EUTRA_r16, physCellId_r16),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_EUTRA_PhysCellId,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"physCellId-r16"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct PCI_ARFCN_EUTRA_r16, carrierFreq_r16),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ARFCN_ValueEUTRA,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"carrierFreq-r16"
		},
};
static const ber_tlv_tag_t asn_DEF_PCI_ARFCN_EUTRA_r16_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_PCI_ARFCN_EUTRA_r16_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* physCellId-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* carrierFreq-r16 */
};
asn_SEQUENCE_specifics_t asn_SPC_PCI_ARFCN_EUTRA_r16_specs_1 = {
	sizeof(struct PCI_ARFCN_EUTRA_r16),
	offsetof(struct PCI_ARFCN_EUTRA_r16, _asn_ctx),
	asn_MAP_PCI_ARFCN_EUTRA_r16_tag2el_1,
	2,	/* Count of tags in the map */
	0, 0, 0,	/* Optional elements (not needed) */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_PCI_ARFCN_EUTRA_r16 = {
	"PCI-ARFCN-EUTRA-r16",
	"PCI-ARFCN-EUTRA-r16",
	&asn_OP_SEQUENCE,
	asn_DEF_PCI_ARFCN_EUTRA_r16_tags_1,
	sizeof(asn_DEF_PCI_ARFCN_EUTRA_r16_tags_1)
		/sizeof(asn_DEF_PCI_ARFCN_EUTRA_r16_tags_1[0]), /* 1 */
	asn_DEF_PCI_ARFCN_EUTRA_r16_tags_1,	/* Same as above */
	sizeof(asn_DEF_PCI_ARFCN_EUTRA_r16_tags_1)
		/sizeof(asn_DEF_PCI_ARFCN_EUTRA_r16_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_PCI_ARFCN_EUTRA_r16_1,
	2,	/* Elements count */
	&asn_SPC_PCI_ARFCN_EUTRA_r16_specs_1	/* Additional specs */
};

